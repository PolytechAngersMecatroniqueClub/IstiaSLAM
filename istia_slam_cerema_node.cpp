#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Path.h>

// to handle the mapping
#include "src/IstiaSlam.h"

// this is for the non blocking input
#include <signal.h>
#include <termios.h>
#include <stdio.h>

// this is for the file operations
#include <fstream>
#include <string.h>


// values for the map building
int maxcost2add;       // max cost for a LiDAR scan to be added
int cpt_no_change_val; // number of iteration where the pose does not change
int watchdog;          // maximal number of iteration

// to handle the illumance "map" file
std::ofstream ifile;  // the file
std::string filename; // the file name

// to save the sensor values
std::vector<sensor_msgs::LaserScan> scan_vector(0);       // to save the LiDAR measurements
std::vector<sensor_msgs::Illuminance> ill_full_vector(0); // to save the illuminance full spectrum measurements
std::vector<sensor_msgs::Illuminance> ill_ired_vector(0); // to save the illuminance infrared measurements

IstiaSlam* istiaslam;           // to handle the nelder and mead, and the maps

geometry_msgs::Pose2D pose;     // the current pose (according to the current LiDAR scan and current map)
geometry_msgs::Pose2D old_pose; // the last pose
nav_msgs::Path path;            // the path (trajectory) of the robot
sensor_msgs::PointCloud cloud;  // for debugging purpose

// this function allows to quickly change the mapping parameters without modifying other files
// this should be done with the parameter server... work in progress...
void init_parameters(){
    // -------------------------------------
    // definition of the parameters values
    // -------------------------------------
    // parameters for the map building
    const int cst_maxcost2add = 1500;    // max cost for a LiDAR scan to be added
    const int cst_cpt_no_change_val = 2; // number of iteration where the pose does not change
    const int cst_watchdog = 50;         // maximal number of iteration

    // parameters for the nelder and mead
    const int cst_nb_ite_max = 10;            // number of iterations

    // parameters for the maps
    const float cst_resolution = 0.02;                         // m/cells
    const int cst_height = (int)(150/cst_resolution);           // nb cells
    const int cst_width = (int)(150/cst_resolution);            // nb cells
    const int cst_cost_stamp_radius = 15;                      // nb cells

    const int cst_max_belief = 100;        // max value in the probability map
    const int cst_min_belief = 0;          // min value in the probability map
    const int cst_add_belief = 25;         // the value we add when detected an obstacle in the probability map
    const int cst_rem_belief = 10;         // the value we remove when detected a free zone in the probability map
    const int cst_threshold_belief = 50;   // the value over the one we consider the cell as an obstacle, a free zone otherwise

    // -------------------------------------
    // setting the parameters
    // -------------------------------------
    maxcost2add = cst_maxcost2add;
    cpt_no_change_val = cst_cpt_no_change_val;
    watchdog = cst_watchdog;
    istiaslam->_nb_ite_max = cst_nb_ite_max;
    istiaslam->_map.init_maps(cst_height, cst_width, cst_resolution, cst_cost_stamp_radius);
    istiaslam->_map._maxBelief = cst_max_belief;
    istiaslam->_map._minBelief = cst_min_belief;
    istiaslam->_map._addBelief = cst_add_belief;
    istiaslam->_map._remBelief = cst_rem_belief;
    istiaslam->_map._thresholdBelief = cst_threshold_belief;
}


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

// for debugging purpose, the idea is to convert a scan to a point cloud according to a pose
void update_cloud(int idscan, const geometry_msgs::Pose2D& ps){
    if(idscan >= scan_vector.size()){
        ROS_ERROR("scan not available");
        return;
    }

    cloud.points.clear();
    sensor_msgs::LaserScan scan = scan_vector[idscan];
    cloud.header.frame_id = "map_link";

    unsigned int i, size = scan.ranges.size();
    double angle = scan.angle_min;

    for(i = 0; i < size; i++){
        if(scan.ranges[i] == 0 || scan.ranges[i] >= INFINITY || scan.ranges[i] < 0.2f || scan.ranges[i] >= 100.0f){
            // I do not know why sometimes we go here... maybe it is because of bad scans...
        }
        else{
            geometry_msgs::Point32 point;
            point.x = scan.ranges[i] * cos(angle + ps.theta) + ps.x;
            point.y = scan.ranges[i] * sin(angle + ps.theta) + ps.y;
            cloud.points.push_back(point);
        }
        angle += scan.angle_increment;
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
    filename = "/home/remy/Documents/rosbags/outputs/illuminance_"+std::to_string(ros::Time::now().toSec())+".csv";
    ifile.open(filename, std::ofstream::out);
    ifile << "Illuminance map " << std::endl;
    ifile << "x (m)\ty (m)\tFull\tIR\ttime difference (ms)" << std::endl;
    ifile.close();

    //initialisation of the LiDAR pose
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    istiaslam->add_2_map(scan_vector[i], pose); // we directly add the first LiDAR data to the map as the origin of the map
    i++;
    

    // to handle the loops
    int watch_dog = 0;
    int cpt_no_change = 0;

    // to handle the time computation
    double time_start = 0;   // iteration starting time
    double time_end = 0;     // iteration endding time
    double time_delta = 0;   // iteration time (end - start)
    double time_max = 0;     // maximal iteration time
    double time_min = 0;     // minimal iteration time
    double time_sum = 0;     // the sum of all the iteration time (to compute the average)
    double time_average = 0; // the average iteration time
    int time_cpt = 0;        // number of added iteration time (to process the average)

    // we loop over the lidar measurements
    while(i<nbmsrlidar && !brk){
        time_start = ros::Time::now().toSec(); // saving the iteration starting time
        watch_dog = 0; // initialiaztion of the watchdog (not to loop too long)
        do{
            cpt_no_change = 0; // this count the number of identical pose computed
            while(cpt_no_change < cpt_no_change_val){ // we are waiting for the nelder and mead to return the same value several times
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
        }while(istiaslam->get_cost(scan_vector[i], pose) > maxcost2add && watch_dog < watchdog); // if we did too many iterations or the new pose has a valid cost

        // if we did not break the loop because of the watchdog
        if(watch_dog < watchdog){
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
            ROS_ERROR("scan not added! - current cost: %2.2f", istiaslam->get_cost(scan_vector[i], pose));
            update_cloud(i, pose);
            return -1;
        }
        time_end = ros::Time::now().toSec(); // saving the iteration endding time
        time_delta =  time_end - time_start; // computing the iteration time
        if(i==1){ // first nelder and mead iteration, initialization of the min and max values
            time_min = time_delta;
            time_max = time_delta;
        }else{
            // saving the min and max iteration time
            if(time_delta < time_min) time_min = time_delta;
            if(time_delta > time_max) time_max = time_delta;
        }
        // computing the sum of all the iteration time
        time_sum += time_delta; 
        time_cpt ++; // number of added iteration time (we could use i...)
        i++; // go to the next scan
    }
    // we clean the vectors, so we can run a map computation we new data again
    scan_vector.clear();
    ill_full_vector.clear();
    ill_ired_vector.clear();
    time_average = time_sum/time_cpt; // computation of the time average
    ROS_INFO("Time results (ms):\n\tmix : %2.2f\n\tmax : %2.2f\n\taverage : %2.2f", time_min*1000, time_max*1000, time_average*1000);
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
    // for debugging purpose
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("/pointcloud_msg", 1000);
    //  Build SLAM object
    istiaslam = new IstiaSlam(); // It can also publish the map

    ros::Rate loop_rate(10); // frequency of the ros loop (in Hz)

    // to override the terminal configuration to be non blocking
    struct termios term_settings;
    SetKeyboardNonBlock(&term_settings);
    ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
    bool conf_overrided = true;
    int result; // for debugging the map building

    init_parameters(); // initialization of the parameters

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
                    if(buildmap() == 1){
                        ROS_INFO("Maps done!");
                    }else{
                        cloud_pub.publish(cloud);
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
                case 'c':
                    istiaslam->publish_cost_map();
                    ROS_INFO("CostMap published");
                    break;
                case 'l':
                    cloud_pub.publish(cloud);
                    ROS_INFO("Error Pointcould published");
                    break;
                case 'o':
                    istiaslam->probability_map_to_csv("/home/remy/Documents/rosbags/outputs/probability_"+std::to_string(ros::Time::now().toSec())+".csv", "\t");
                    ROS_INFO("Map saved to csv file");
                    break;
                case -1:
                    // no key has been pressed
                    break;
                default:
                    // one not defined key has been pressed
                    if(conf_overrided){
                        ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
                    }
                    ROS_WARN("Key pressed (%d) not valid", c);
                    ROS_INFO("available commands:\n'q': change keyboard configuration\n's': build map\n'm': publish map\n'c': publish costmap\n'p': publish path\n'o': save probability map to csv file");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

