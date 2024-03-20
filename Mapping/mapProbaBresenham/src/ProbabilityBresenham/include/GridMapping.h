#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/filters/passthrough.h>
#include <pcl-1.10/pcl/filters/voxel_grid.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include "Map.h"
#include "VoxelVisualizer.h"
#include <thread>
class GridMapping{
public:
    ros::NodeHandle                             handler;

    GridMapping() = delete;
    GridMapping(ros::NodeHandle& nodeHandler,int mapSizeX,int mapSizeY, int mapSizeZ,double resolution,tf2_ros::Buffer& Buffer,tf2_ros::TransformListener& Listener);
    void initMap(int sizeOfMapX,int sizeOfMapY, int sizeOfMapZ, double resolution, int startX, int startY, int startz);
    void initSubscribersAdvertisers();
    void RealsenseCallback(const sensor_msgs::PointCloud2 msg);
    void initVisualizer(ros::NodeHandle& handler,double size);

    void updateMap(pcl::PointCloud<pcl::PointXYZ>& points,pcl::PointCloud<pcl::PointXYZ>& fileredCloud,geometry_msgs::TransformStamped& transform);
    void mappingCycle();
    void sendVisualization();    

private:
    ros::Subscriber                             RealSenseSubscriber;
    Map                                         map;
    double                                      mapResolution;
    int                                         mapSizeX;
    int                                         mapSizeY;
    int                                         mapSizeZ;   
    double                                         maxDepth;
    double                                       pOccupied;
    double                                       pFree; 
    tf2_ros::Buffer&                             tfBuffer;
    tf2_ros::TransformListener&                  tfListener;
    VoxelVisualizer                             visualizer;
    std::thread                                 visualizerThread;
};