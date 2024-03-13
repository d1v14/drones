#pragma once
#include <vector>
#include <geometry_msgs/TransformStamped.h>

typedef std::vector<std::vector<std::vector<double>>> vector3d_t;
class Map{
public:
    Map(int sizeOfMapX,int sizeOfMapY, int sizeOfMapZ, double resolution);
    Map() = default;
    std::vector<int> getCageCoordinates(double x,double y, double z);

    void setValueToMap(double x, double y, double z, float value);
    void clear();
    void Bresenham(std::vector<int> values,double value);
    std::vector<double> convertMapPointsToVoxel(int x, int y, int z);
    const vector3d_t& getMap();
private:
    double bresCageP;
    double bresCageL;
    int sizeOfMapX;
    int sizeOfMapY;
    int sizeOfMapZ;
    int startXinCages;
    int startYinCages;
    int startZinCages;
    int camDepth;
    double startXinCoordinates;
    double startYinCoordinates;
    double startZinCoordinates;
    double resolution;
    vector3d_t grid;
    };