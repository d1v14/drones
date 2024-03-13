#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
class VoxelVisualizer{
public:
    VoxelVisualizer() = default;
    VoxelVisualizer(ros::NodeHandle& handler,double size);
    void clear();
    void addMarker(double x, double y, double z,double value);

    void pubUpdate();

private:
    double                                  size;
    ros::NodeHandle                         handler;
    ros::Publisher                          voxelPublisher;
    visualization_msgs::MarkerArray         markerArray;
    visualization_msgs::Marker              marker;
};