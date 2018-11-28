#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>


// this is for the non blocking input
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include "src/IstiaSlam.h"

unsigned int id_scan = 0;
unsigned int step_scan = 5;

double rotation_step = 0.01;
double translation_step = 0.01;

std::vector<sensor_msgs::LaserScan> scan_vector(0);
sensor_msgs::PointCloud cloud;
IstiaSlam* istiaslam;
geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D old_pose;

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
    //ROS_INFO("new scan added, current length %d", scan_vector.size());
}


void update_cloud(ros::Publisher& pub){
    if(id_scan >= scan_vector.size()){
        ROS_ERROR("scan not available");
        id_scan = scan_vector.size();
        return;
    }
    cloud.points.clear();
    sensor_msgs::LaserScan scan = scan_vector[id_scan];
    cloud.header.frame_id = "map_link";

    unsigned int i, size = scan.ranges.size();
    double angle = scan.angle_min;

    for(i = 0; i < size; i++){
        if(scan.ranges[i] == 0 || scan.ranges[i] >= INFINITY || scan.ranges[i] < 0.2f || scan.ranges[i] >= 100.0f){
            // I do not know why sometimes we go here... maybe it is because of bad scans...
        }
        else{
            geometry_msgs::Point32 point;
            point.x = scan.ranges[i] * cos(angle + pose.theta) + pose.x;
            point.y = scan.ranges[i] * sin(angle + pose.theta) + pose.y;
            cloud.points.push_back(point);
        }
        angle += scan.angle_increment;
    }

    pub.publish(cloud);
}

void updateMaps(){
    if(id_scan >= scan_vector.size()){
        ROS_ERROR("scan not available");
        id_scan = scan_vector.size();
        return;
    }
    istiaslam->add_2_map(scan_vector[id_scan], pose);
    istiaslam->publish_probability_map();
}

int updatePose(ros::Publisher& pub){
    if(id_scan >= scan_vector.size()){
        ROS_ERROR("scan not available");
        id_scan = scan_vector.size();
        return -1;
    }
    int cpt = 0;

    geometry_msgs::Pose2D pose_back(pose);
    int watch_dog = 0;
    do{
        while(cpt < 10){
            old_pose = pose;
            pose = istiaslam->nelder_mead(scan_vector[id_scan], pose);
            if(old_pose.x == pose.x && old_pose.y == pose.y && old_pose.theta == pose.theta){
                cpt++;
            }else{
                cpt=0;
            }
        }
        watch_dog++;
    }while(istiaslam->get_cost(scan_vector[id_scan], pose) > 7000 && watch_dog < 50);
    ROS_INFO("cost: %2.2f", istiaslam->get_cost(scan_vector[id_scan], pose));
    update_cloud(pub);
    if(watch_dog==50){
        return 0;
    }else{
        return 1;
    }
}



//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // Set up ROS.
    ros::init(argc, argv, "istia_slam__cerema_node",1);
    ros::NodeHandle node;


    ros::Subscriber sub_lidar = node.subscribe("/scan", 1000, addLidarScan);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("/pointcloud_msg", 1000);

    //  Build SLAM object :
    istiaslam = new IstiaSlam();

    ros::Rate loop_rate(10);

    struct termios term_settings;
    SetKeyboardNonBlock(&term_settings);
    ROS_WARN("The terminal configuration is overrided - press 'Q' to get initial configuration");
    while(ros::ok())
    {
        char c = getchar();   // call your non-blocking input function
        if(c!=-1){
            switch(c){
                case 'p': // 'p'
                    if(id_scan > step_scan){
                        id_scan -= step_scan;
                        update_cloud(cloud_pub);
                    }
                    break;
                case 'n': // 'n'
                    id_scan += step_scan;
                    update_cloud(cloud_pub);
                    break;
                case 'Q': // 'get the terminal back in track'
                    RestoreKeyboardBlocking(&term_settings);
                    ROS_WARN("The terminal is back to normal");
                    break;
                case -1:
                    // no key has been pressed
                    break;
                case 'm':
                    updateMaps();
                    break;
                case 'l':
                    updatePose(cloud_pub);
                    break;
                case 'c':
                    istiaslam->publish_cost_map();
                    break;
                case 'O':
                    pose.y += translation_step;
                    update_cloud(cloud_pub);
                    break;
                case 'L':
                    pose.y -= translation_step;
                    update_cloud(cloud_pub);
                    break;
                case 'K':
                    pose.x -= translation_step;
                    update_cloud(cloud_pub);
                    break;
                case 'M':
                    pose.x += translation_step;
                    update_cloud(cloud_pub);
                    break;
                case 'P':
                    pose.theta += rotation_step;
                    update_cloud(cloud_pub);
                    break;
                case 'I':
                    pose.theta -= rotation_step;
                    update_cloud(cloud_pub);
                    break;
                case ' ':
                    id_scan += step_scan;
                    if(updatePose(cloud_pub)==1){
                        updateMaps();
                    }else ROS_INFO("Failed to update the pose");
                    break;
                default:
                    //send_smth = false;
                    ROS_WARN("The terminal configuration is overrided - press 'Q' to get initial configuration");
                    ROS_INFO("Key pressed: %d", c);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

