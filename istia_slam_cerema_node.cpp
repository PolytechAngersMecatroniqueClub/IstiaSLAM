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

#include "src/IstiaSlam.h"

std::ofstream ifile; // to handle the illumance "map" file

unsigned int id_scan = 0;
unsigned int step_scan = 1;

double rotation_step = 0.01;
double translation_step = 0.01;

double limit_score = 3000;
int maxcost2add = 1500;

std::vector<sensor_msgs::LaserScan> scan_vector(0);
std::vector<sensor_msgs::Illuminance> ill_full_vector(0);
std::vector<sensor_msgs::Illuminance> ill_ired_vector(0);

IstiaSlam* istiaslam;
geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D old_pose;
nav_msgs::Path path;

// this is for the non blocking input
void RestoreKeyboardBlocking(struct termios *initial_settings){
    tcsetattr(0, TCSANOW, initial_settings);
}

// this is for the non blocking input
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


void addLidarScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    scan_vector.push_back(*msg);
    ROS_INFO("new scan added, current length %d", scan_vector.size());
}

void addIllFull(const sensor_msgs::Illuminance::ConstPtr& msg){
    ill_full_vector.push_back(*msg);
    ROS_INFO("new full spectrum illuminance added, current length %d", ill_full_vector.size());
}

void addIlliRed(const sensor_msgs::Illuminance::ConstPtr& msg){
    ill_ired_vector.push_back(*msg);
    ROS_INFO("new infra red illuminance added, current length %d", ill_ired_vector.size());
}

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
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

void updatepath(){
    path.header.frame_id = "map_link";

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map_link";
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.orientation = toQuaternion(0.0f,0.0f,pose.theta);

    path.poses.push_back(pose_stamped);
}

void update_illuminance(int idscan){
    // if this is the first or the last scan, no need to look for illuminance data
    if(idscan == 0 || idscan >= (scan_vector.size()-2) ){
        return;
    }

    // check if a full spectrum illuminance data correponds to the pose
    // 
    for (int i=0; i<ill_full_vector.size(); i++){
        if(ill_full_vector[i].header.stamp > scan_vector[idscan].header.stamp && ill_full_vector[i].header.stamp < scan_vector[idscan+1].header.stamp){
            ifile.open("illuminance_map.csv", std::ofstream::app);
            ifile << "full\t" << pose.x << "\t"<< pose.y << "\t" << ill_full_vector[i].illuminance << "\t" << (ill_full_vector[i].header.stamp - scan_vector[idscan].header.stamp).toSec()*100 << std::endl;
            ifile.close();
            ROS_INFO("Illuminance full spectrum added");
        }
    }

    for (int i=0; i<ill_ired_vector.size(); i++){
        if(ill_ired_vector[i].header.stamp > scan_vector[idscan].header.stamp && ill_ired_vector[i].header.stamp < scan_vector[idscan+1].header.stamp){
            ifile.open("illuminance_map.csv", std::ofstream::app);
            ifile << "ired\t" << pose.x << "\t"<< pose.y << "\t" << ill_ired_vector[i].illuminance << "\t" << (ill_ired_vector[i].header.stamp - scan_vector[idscan].header.stamp).toSec()*100 << std::endl;
            ifile.close();
            ROS_INFO("Illuminance Infrared added");
        }
    }

}

int buildmap(){
    bool brk = false;
    int i=0;
    int nbmsrlidar = scan_vector.size();
    if(nbmsrlidar == 0){
        ROS_ERROR("No lidar measurements to process");
        return -1;
    }

    ifile.open("illuminance_map.csv", std::ofstream::out);
    ifile << "Illuminance map - " << ros::Time::now() << std::endl;
    ifile << "Illuminance type\tx (m)\ty (m)\tvalue\ttime difference (ms)" << std::endl;
    ifile.close();

    //initialisation
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    istiaslam->add_2_map(scan_vector[i], pose);

    while(i<nbmsrlidar && !brk){
        geometry_msgs::Pose2D pose_back(pose);
        int watch_dog = 0;
        do{
            int cpt_no_change = 0;
            while(cpt_no_change < 5){
                old_pose = pose;
                pose = istiaslam->nelder_mead(scan_vector[i], pose);
                if(old_pose.x == pose.x && old_pose.y == pose.y && old_pose.theta == pose.theta){
                    cpt_no_change++;
                }else{
                    cpt_no_change=0;
                }
            }
            watch_dog++;
        }while(istiaslam->get_cost(scan_vector[i], pose) > maxcost2add && watch_dog < 50);

        if(watch_dog < 50){
            istiaslam->add_2_map(scan_vector[i], pose);
            update_illuminance(i);
            updatepath();
            ROS_INFO("scan processed %d/%d", i+1, nbmsrlidar);
        }else{
            ROS_ERROR("scan not added!");
            return -1;
        }
        i++;
    }
    return 1;
}


//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // Set up ROS.
    ros::init(argc, argv, "istia_slam_cerema_node",1);
    ros::NodeHandle node;

    ros::Subscriber sub_lidar    = node.subscribe("/scan", 1000, addLidarScan);
    ros::Subscriber sub_ill_full = node.subscribe("/tsl2561/full_spectrum", 1000, addIllFull);
    ros::Subscriber sub_ill_ired = node.subscribe("/tsl2561/infrared", 1000, addIlliRed);

    ros::Publisher path_pub = node.advertise<nav_msgs::Path>("/path_msg", 1000);
    //  Build SLAM object :
    istiaslam = new IstiaSlam();

    ros::Rate loop_rate(10);

    struct termios term_settings;
    SetKeyboardNonBlock(&term_settings);
    ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
    while(ros::ok())
    {
        char c = getchar();   // call your non-blocking input function
        if(c!=-1){
            switch(c){
                case 'q': // 'get the terminal back in track'
                    RestoreKeyboardBlocking(&term_settings);
                    ROS_WARN("The terminal is back to normal");
                    break;
                case 's':
                    ROS_INFO("Processing LiDAR scan...");
                    if(buildmap()==1){
                        ROS_INFO("Maps done!");
                    }
                    break;
                case 'p':
                    path_pub.publish(path);
                    ROS_INFO("path published");
                    break;
                case 'm':
                    istiaslam->publish_probability_map();
                    ROS_INFO("map published");
                    break;
                case -1:
                    // no key has been pressed
                    break;
                default:
                    //send_smth = false;
                    ROS_WARN("The terminal configuration is overrided - press 'q' to get initial configuration");
                    ROS_INFO("Key pressed: %d", c);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

