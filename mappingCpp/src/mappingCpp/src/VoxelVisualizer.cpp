#include "VoxelVisualizer.h"

VoxelVisualizer::VoxelVisualizer(ros::NodeHandle& handler,double size){
    this->handler = handler;
    voxelPublisher = handler.advertise<visualization_msgs::MarkerArray>("voxel_marker_array",1);
    markerArray = visualization_msgs::MarkerArray();
    this->size = size;
}
void VoxelVisualizer::clear(){
    markerArray.markers.clear();
}
void VoxelVisualizer::addMarker(double x, double y, double z,double value){
    visualization_msgs::Marker addMarker;
    addMarker.header.frame_id = "map";
    addMarker.type = visualization_msgs::Marker::CUBE;
    addMarker.action = visualization_msgs::Marker::ADD;
    addMarker.pose.position.x = x;
    addMarker.pose.position.y = y;
    addMarker.pose.position.z = z;
    addMarker.pose.orientation.x = 0;
    addMarker.pose.orientation.y = 0;
    addMarker.pose.orientation.z = 0;
    addMarker.pose.orientation.w = 1;
    addMarker.scale.x = size;
    addMarker.scale.y = size;
    addMarker.scale.z = size;
    addMarker.color.r = double(value);
    addMarker.color.g = double(1- value);
    addMarker.color.b = double(value * 0.5);
    addMarker.color.a = value;

    markerArray.markers.push_back(addMarker);
}
void VoxelVisualizer::pubUpdate(){
    for(int i=0; i<markerArray.markers.size();i++)
    {
        markerArray.markers.at(i).id = i;
    }
    voxelPublisher.publish(markerArray);
}