#include <ros/ros.h>
#include <ros/param.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

#include "src/IstiaSlam.h"

double rotation_step = 0.01;
double translation_step = 0.01;
int publishing_map_rate = 15;
int nb_scans_added = 0;
int max_cost_2_add = 5000;

bool has_been_initialized = false;

ros::Publisher* p_path_pub;
ros::Publisher* p_cloud_pub;
ros::Publisher* p_lidar_pub;

sensor_msgs::PointCloud cloud;
IstiaSlam* istiaslam;
geometry_msgs::Pose2D pose;
geometry_msgs::Pose2D old_pose;
nav_msgs::Path path;


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

void update_tf(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.x, pose.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map_link", "/laser"));
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
    p_path_pub->publish(path);
}

void addLidarScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    sensor_msgs::LaserScan new_msg;
    new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();
    p_lidar_pub->publish(new_msg);
    if (!has_been_initialized){
        has_been_initialized = true;
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
        istiaslam->add_2_map(*msg, pose);
    }else{
        geometry_msgs::Pose2D pose_back(pose);
        int watch_dog = 0;
        do{
            int cpt_no_change = 0;
            while(cpt_no_change < 5){
                old_pose = pose;
                pose = istiaslam->nelder_mead(*msg, pose);
                if(old_pose.x == pose.x && old_pose.y == pose.y && old_pose.theta == pose.theta){
                    cpt_no_change++;
                }else{
                    cpt_no_change=0;
                }
            }
            watch_dog++;
        }while(istiaslam->get_cost(*msg, pose) > max_cost_2_add && watch_dog < 50);

        if(watch_dog < 50){
            istiaslam->add_2_map(*msg, pose);
            updatepath();
            update_tf();
        }else{
            ROS_ERROR("scan not added!");
        }
    }
    nb_scans_added ++;
    if(nb_scans_added%publishing_map_rate==0){
        istiaslam->publish_probability_map();
    }
}



//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // Set up ROS.
    ros::init(argc, argv, "istia_slam_real_time_node",1);
    ros::NodeHandle node;


    ros::Subscriber sub_lidar = node.subscribe("/scan", 5, addLidarScan);
    ros::Publisher path_pub = node.advertise<nav_msgs::Path>("/path_msg", 1000);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("/pointcloud_msg", 1000);
    ros::Publisher lidar_pub = node.advertise<sensor_msgs::LaserScan>("/updated_scan_msg", 1000);
    p_path_pub = &path_pub;
    p_cloud_pub = &cloud_pub;
    p_lidar_pub = &lidar_pub;

    //  Build SLAM object :
    istiaslam = new IstiaSlam();

    ros::Rate loop_rate(1);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

