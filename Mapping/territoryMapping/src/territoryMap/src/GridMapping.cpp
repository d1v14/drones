#include "GridMapping.h"

GridMapping::GridMapping(ros::NodeHandle& nodeHandler,int mapSizeX,int mapSizeY, int mapSizeZ,double resolution,tf2_ros::Buffer& Buffer,tf2_ros::TransformListener& Listener):tfBuffer(Buffer),tfListener(Listener),map(mapSizeX,mapSizeY,mapSizeZ,resolution){
        handler = nodeHandler;
        mapResolution = resolution;
        this->mapSizeX = mapSizeX;
        this->mapSizeY = mapSizeY;
        this->mapSizeZ = mapSizeZ;
        this->maxDepth = 8;
        initVisualizer(handler,resolution); 
        visualizerThread = std::thread(&GridMapping::mappingCycle,this);
        visualizerThread.detach();
        initSubscribersAdvertisers();       
    }


void GridMapping::initVisualizer(ros::NodeHandle& handler, double size){
    visualizer = VoxelVisualizer(handler,size);
}

void GridMapping::mappingCycle(){
    ros::Rate rate = ros::Rate(1);
    while(ros::ok()){
        sendVisualization();
        rate.sleep();
    }
}
void GridMapping::sendVisualization(){
    visualizer.clear();
    vector3d_t temp = map.getMap();
    for(int x = 0; x<mapSizeX;x++){
        for(int y = 0; y<mapSizeY;y++){
            for(int z = 0; z<mapSizeZ;z++){
                if(temp[x][y][z]==1){
                    std::vector<double> answer = map.convertMapPointsToVoxel(x,y,z);
                    visualizer.addMarker(answer[0],answer[1],answer[2]);
                    }
            }
        }
    }
    visualizer.pubUpdate();
}

void GridMapping::initSubscribersAdvertisers(){
        RealSenseSubscriber = this->handler.subscribe<sensor_msgs::PointCloud2>("/realsense_d435_depth/points",10,&GridMapping::RealsenseCallback,this);
}

void GridMapping::RealsenseCallback(const sensor_msgs::PointCloud2  msg){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloudFiltered;
        pcl::fromROSMsg(msg,cloud);
        pcl::PassThrough<pcl::PointXYZ> passthroughFilter;
        passthroughFilter.setInputCloud(cloud.makeShared());
        passthroughFilter.setFilterLimits(0.1,8.0);
        passthroughFilter.setFilterFieldName("z");
        passthroughFilter.filter(cloudFiltered);
        pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
        voxelGridFilter.setInputCloud(cloudFiltered.makeShared());
        voxelGridFilter.setLeafSize(mapResolution,mapResolution,mapResolution);
        pcl::PointCloud<pcl::PointXYZ> downsampled;
        voxelGridFilter.filter(downsampled);
        sensor_msgs::PointCloud2 outputCloud;
        pcl::toROSMsg(downsampled,outputCloud);
        outputCloud.header = msg.header;
        sensor_msgs::PointCloud2 transformedOutputCloud;
        geometry_msgs::TransformStamped transformation;
        try
        {
            transformation = tfBuffer.lookupTransform("map",msg.header.frame_id,msg.header.stamp,ros::Duration(4.0));
            tf2::doTransform(outputCloud,transformedOutputCloud,transformation);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        pcl::PointCloud<pcl::PointXYZ> pointsOfMap;
        pcl::fromROSMsg(transformedOutputCloud,pointsOfMap);
        updateMap(pointsOfMap,downsampled,transformation);
    }

void GridMapping::updateMap(pcl::PointCloud<pcl::PointXYZ> points,pcl::PointCloud<pcl::PointXYZ> filteredCloud,geometry_msgs::TransformStamped transform){
        for(int i =0; i<points.size();i++){
            //if(filteredCloud[i].z < double(maxDepth))
                map.setValueToMap(points[i].x,points[i].y,points[i].z,1);
            // else
            //     map.setValueToMap(points[i].x,points[i].y,points[i].z,0);
        }   

    }