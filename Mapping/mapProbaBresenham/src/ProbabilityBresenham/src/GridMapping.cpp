#include "GridMapping.h"

GridMapping::GridMapping(ros::NodeHandle& nodeHandler,int mapSizeX,int mapSizeY, int mapSizeZ,double resolution,tf2_ros::Buffer& Buffer,tf2_ros::TransformListener& Listener):tfBuffer(Buffer),tfListener(Listener),map(mapSizeX,mapSizeY,mapSizeZ,resolution){
        handler = nodeHandler;
        maxDepth = 10;
        pOccupied = 0.99;
        mapResolution = resolution;
        this->mapSizeX = mapSizeX;
        this->mapSizeY = mapSizeY;
        this->mapSizeZ = mapSizeZ;
        this->pFree = 0.45;
        this->pOccupied = pOccupied;
        //initMap(mapSizeX,mapSizeY,mapSizeZ,mapResolution,mapSizeX/2,mapSizeY/2,1);
        initVisualizer(handler,resolution); 
        visualizerThread = std::thread(&GridMapping::mappingCycle,this);
        visualizerThread.detach();
        initSubscribersAdvertisers();
        // auto future1 = std::async(std::launch::async,&GridMapping::mappingCycle,this);
    }

void GridMapping::initMap(int sizeOfMapX,int sizeOfMapY, int sizeOfMapZ, double resolution, int startX, int startY, int startz){
      //  map = Map(sizeOfMapX,sizeOfMapY,sizeOfMapZ,resolution,startX,startY,startz);
     }

void GridMapping::initVisualizer(ros::NodeHandle& handler, double size){
    visualizer = VoxelVisualizer(handler,size);
}

void GridMapping::mappingCycle(){
    ros::Rate rate = ros::Rate(0.1);

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
                if(temp[x][y][z] > double(0.6))
                {
                    std::vector<double> answer = map.convertMapPointsToVoxel(x,y,z);
                    visualizer.addMarker(answer[0],answer[1],answer[2],temp[x][y][z]);
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
        pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
        voxelGridFilter.setInputCloud(cloud.makeShared());
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

void GridMapping::updateMap(pcl::PointCloud<pcl::PointXYZ>& points,pcl::PointCloud<pcl::PointXYZ>& filteredCloud,geometry_msgs::TransformStamped& transform){
        //map.clear();
        for(int i =0; i<points.size();i++){
            if(filteredCloud[i].z < maxDepth)
                map.setValueToMap(points[i].x,points[i].y,points[i].z,pOccupied);
            //  else{
            //     // sensor_msgs::PointCloud2 temp;
            //     // sensor_msgs::PointCloud2 result;
            //     // pcl::PointCloud<pcl::PointXYZ> cloudCopy = filteredCloud;
            //     // pcl::PointCloud<pcl::PointXYZ> cloudCopy2;
            //     // cloudCopy[i].z = maxDepth;
            //     // pcl::toROSMsg(cloudCopy,temp);
            //     // tf2::doTransform(temp,result,transform);
            //     // pcl::fromROSMsg(result,cloudCopy2);
            //     // map.setValueToMap(cloudCopy2[i].x,cloudCopy2[i].y,cloudCopy2[i].z,pFree); 
            //     map.setValueToMap(points[i].x,points[i].y,points[i].z,pFree);
            // }
        }   

    }