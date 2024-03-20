#include "VoxelVisualizer.h"

VoxelVisualizer::VoxelVisualizer(ros::NodeHandle& handler,double size){
    this->handler = handler;
    voxelPublisher = handler.advertise<visualization_msgs::MarkerArray>("voxel_marker_array",1);
    markerArray = visualization_msgs::MarkerArray();
    this->size = size;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
}
void VoxelVisualizer::clear(){
    marker.points.clear();
}
void VoxelVisualizer::addMarker(double x, double y, double z){

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    marker.points.push_back(point);
}
void VoxelVisualizer::pubUpdate(){
    markerArray.markers.push_back(marker);
    voxelPublisher.publish(markerArray);
}