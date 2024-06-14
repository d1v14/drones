#include <ros/ros.h>
#include "GridMapping.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
int main(int argc, char **argv){
    ros::init(argc,argv,"mapping_node");
    ROS_INFO("Mapping Node started");
    ros::NodeHandle handle;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    GridMapping gridmapping(handle,1000,1000,100,0.1,buffer,listener);
    ros::spin();
}