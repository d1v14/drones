#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <vector>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"mapping_node");
    ros::NodeHandle handler;
    ros::Rate rate(60);
    ROS_INFO("Node mapping");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    rate.sleep();
    return 0;
}
