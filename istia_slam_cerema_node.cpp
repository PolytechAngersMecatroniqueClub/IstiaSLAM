#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Path.h>

// this is for the non blocking input
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <fstream>

#include <string.h>

#include "src/IstiaSlam.h"

// constants values for the map building
const int cst_maxcost2add = 1500;    // max cost for a LiDAR scan to be added
const int cst_cpt_no_change_val = 2; // number of iteration where the pose does not change
const int cst_watchdog = 50;         // maximal number of iteration

// to handle the illumance "map" file
std::ofstream ifile;  // the file
std::string filename; // the file name

// to save the sensor values
std::vector<sensor_msgs::LaserScan> scan_vector(0);
std::vector<sensor_msgs::Illuminance> ill_full_vector(0);
std::vector<sensor_msgs::Illuminance> ill_ired_vector(0);

IstiaSlam* istiaslam;           // to handle the nelder and mead, and the maps
geometry_msgs::Pose2D pose;     // the current pose (according to the current LiDAR scan and current map)
geometry_msgs::Pose2D old_pose; // the last pose
nav_msgs::Path path;            // the path (trajectory) of the robot

// this is for the non blocking input
// it restore the keyboard configuration at its initial settings
void RestoreKeyboardBlocking(struct termios *initial_settings){
    tcsetattr(0, TCSANOW, initial_settings);
}

// this is for the non blocking input
// set the keyboard configuration to be non blocking
void SetKeyboardNonBlock(struct termios *initial_settings){
    struct termios new_settings;
    tcgetattr(0,initial_settings);
    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

// to save the LiDAR measurement from a rosbag to a vector
void addLidarScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    scan_vector.push_back(*msg);
    ROS_INFO("new scan added, current length %d", (int)(scan_vector.size()));
}

// to save the Illuminance full spectrum measurements from a rosbag to a vector
void addIllFull(const sensor_msgs::Illuminance::ConstPtr& msg){
    ill_full_vector.push_back(*msg);
    ROS_INFO("new full spectrum illuminance added, current length %d", (int)(ill_full_vector.size()));
}

// to save the Illuminance infrared measurements from a rosbag to a vector
void addIlliRed(const sensor_msgs::Illuminance::ConstPtr& msg){
    ill_ired_vector.push_back(*msg);
    ROS_INFO("new infra red illuminance added, current length %d", (int)(ill_ired_vector.size()));
}

// to transform a pitch/roll/yaw value to a quaternion
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw){
    geometry_msgs::Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x=(t0 * t3 * t4 - t1 * t2 * t5);
    q.y=(t0 * t2 * t5 + t1 * t3 * t4);
    q.z=(t1 * t2 * t4 - t0 * t3 * t5);
    return q;
}

// the save the path of the robot in order to be able to display it
void updatepath(){
    path.header.frame_id = "map_link";

    // the 2D pose needs to be converted into a posestamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map_link";
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.orientation = toQuaternion(0.0f,0.0f,pose.theta);
    // adding the new pose to the path
    path.poses.push_back(pose_stamped);
}

// this function update the illuminance map (csv file with illuminance and corresponding computed positions)
void update_illuminance(int idscan){
    // if this is the first or the last scan, no need to look for illuminance data
    if(idscan == 0 || idscan >= (scan_vector.size()-2) ){
        return;
    }

    // check if a full spectrum illuminance data correponds to the pose
    // as the illuminance data are send at the same time, if the position of the spectrum illuminance data is found, it is the same for the infrared data
    for (int i=0; i<ill_full_vector.size(); i++){
        if(ill_full_vector[i].header.stamp > scan_vector[idscan].header.stamp && ill_full_vector[i].header.stamp < scan_vector[idscan+1].header.stamp){
            ifile.open(filename, std::ofstream::app);
            ifile << pose.x << "\t"<< pose.y << "\t";
            ifile << ill_full_vector[i].illuminance << "\t";
            ifile << ill_ired_vector[i].illuminance << "\t";
            // we compute the time difference between the illuminance timestamp and the lidar timestamp (position)
            ifile << (ill_full_vector[i].header.stamp - scan_vector[idscan].header.stamp).toSec()*100 << std::endl;
            ifile.close();
            ROS_INFO("Illuminance added");
        }
    }
}

// this function uses the lidar vector measurements and the istiaslam variable to build a map
int buildmap(){

    bool brk = false;
    int i=0;
    int nbmsrlidar = scan_vector.size();

    // check if there is LiDAR to process
    if(nbmsrlidar == 0){
        ROS_ERROR("No lidar measurements to process");
        return -1;
    }

    // initialisation of the illuminance csv file to map the illimunance data
    filename = "illuminance"+std::to_string(ros::Time::now().toSec())+".csv";
    ifile.open(filename, std::ofstream::out);
    ifile << "Illuminance map " << std::endl;
    ifile << "x (m)\ty (m)\tFull\tIR\ttime difference (ms)" << std::endl;
    ifile.close();

    //initialisation of the LiDAR pose
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    istiaslam->add_2_map(scan_vector[i], pose); // we directly add the first LiDAR data to the map as the origin of the map

    // to handle the time computation
    int time_start=0, time_end=0, time_min=0, time_max=0, time_sum=0, time_cpt=0;
    double time_average;

    // to handle the loops
    int watch_dog = 0;
    int cpt_no_change = 0;

    // we loop over the lidar measurements
    while(i<nbmsrlidar && !brk){
        watch_dog = 0; // initialiaztion of the watchdog (not to loop too long)
        do{
            cpt_no_change = 0; // this count the number of identical pose computed
            while(cpt_no_change < cst_cpt_no_change_val){ // we are waiting for the nelder and mead to return the same value several times
                old_pose = pose; // we save the previous pose
                pose = istiaslam->nelder_mead(scan_vector[i], pose); // we compute a new pose with nelder and mead optimization
                // we check if this new pose iquals the previous one
                if(old_pose.x == pose.x && old_pose.y == pose.y && old_pose.theta == pose.theta){
                    cpt_no_change++;
                }else{
                    cpt_no_change=0;
                }
            }
            watch_dog++; // update the watch dog not to loop too long
        }while(istiaslam->get_cost(scan_vector[i], pose) > cst_maxcost2add && watch_dog <cst_watchdog); // if we did too many iterations or the new pose has a valid cost

        // if we did not break the loop because of the watchdog
        if(watch_dog < cst_watchdog){
            // we add the new scan to the map
            istiaslam->add_2_map(scan_vector[i], pose);
            // we update the illuminance csv file
            update_illuminance(i);
            // we update the path adding the new pose
            updatepath();
            ROS_INFO("scan processed %d/%d", i+1, nbmsrlidar);
        }else{
            // we break the loop because of the watch dog, meaning that the pose/lidar cost is to high...
            // we do not add the LiDAR scan, we stop the map building
            ROS_ERROR("scan not added!");
            return -1;
        }
        i++; // go to the next scan
    }
    return 1; // map is built correctly
}


// Main
int main(int argc, char **argv)
{
  // Set up ROS.
    ros::init(argc, argv, "istia_slam_cerema_node",1);
    ros::NodeHandle node;

    // suscribers to save the data into the vectors
    ros::Subscriber sub_lidar    = node.subscribe("/scan", 1000, addLidarScan);
    ros::Subscriber sub_ill_full = node.subscribe("/tsl2561/full_spectrum", 1000, addIllFull);
    ros::Subscriber sub_ill_ired = node.subscribe("/tsl2561/infrared", 1000, addIlliRed);

    // to publish the path of the robot
    ros::Publisher path_pub = node.advertise<nav_msgs::Path>("/path_msg", 1000);
    //  Build SLAM object
    istiaslam = new IstiaSlam(); // It can also publish the map

    ros::Rate loop_rate(10); // frequency of the ros loop (in Hz)

    // to override the terminal configuration to be non blocking
    struct termios term_settings;
    SetKeyboardNonBlock(&term_settings);
    ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
    bool conf_overrided = true;

    // main ros loop
    while(ros::ok())
    {
        char c = getchar();   // call your non-blocking input function
        if(c!=-1){
            switch(c){
                case 'q': // get the terminal back in track, or override the configuration again...
                    if(conf_overrided){
                        conf_overrided = false;
                        RestoreKeyboardBlocking(&term_settings);
                        ROS_WARN("The terminal is back to normal - press 'q' to go back into the non blocking configuration");
                    }else{
                        conf_overrided = true;
                        SetKeyboardNonBlock(&term_settings);
                        ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
                    }
                    break;
                case 's':
                    ROS_INFO("Processing LiDAR scan...");
                    if(buildmap()==1){
                        ROS_INFO("Maps done!");
                    }
                    break;
                case 'p':
                    path_pub.publish(path);
                    ROS_INFO("Path published");
                    break;
                case 'm':
                    istiaslam->publish_probability_map();
                    ROS_INFO("Map published");
                    break;
                case -1:
                    // no key has been pressed
                    break;
                default:
                    // one not defined key has been pressed
                    if(conf_overrided){
                        ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
                    }else{
                        ROS_WARN("The terminal configuration is the one by default - press 'q' to override into non blocking configuration");
                    }
                    ROS_INFO("'q':change keyboard configuration, 's':build map, 'm':publish map, 'p':publish path ");
                    ROS_WARN("Key pressed (%d) not valid", c);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

