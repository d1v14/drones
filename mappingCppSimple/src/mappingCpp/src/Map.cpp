
#include "Map.h"
#include "ros/ros.h"
#include <cmath>
Map::Map(int sizeOfMapX ,int sizeOfMapY , int sizeOfMapZ , double resolution){
        this->sizeOfMapX = sizeOfMapX;
        this->sizeOfMapY = sizeOfMapY;
        this->sizeOfMapZ = sizeOfMapZ;
        this->camDepth = 8;
        this->startXinCages = sizeOfMapX/2;
        this->startYinCages =  sizeOfMapY/2;
        this->startZinCages =  1;
        this->resolution = resolution;
        this->bresCageP = 0.45;
        this->bresCageL = log(bresCageP/(1-bresCageP));
        this->startXinCoordinates = double(resolution * startXinCages);
        this->startYinCoordinates = double(resolution * startYinCages);
        this->startZinCoordinates = double(resolution * startZinCages);
        for(int i=0;i<sizeOfMapX;i++)
        { 
            std::vector<std::vector<double>> v2d;
            for(int j=0;j<sizeOfMapY;j++)
            {
            std::vector<double> v1d;
            for(int k=0;k<sizeOfMapZ;k++)
            {
                v1d.push_back(0.5);
            }
            v2d.push_back(v1d);
            }
            grid.push_back(v2d);
        }
}
std::vector<double> Map::convertMapPointsToVoxel(int x, int y, int z){
    std::vector<double> voxelCoord(3);
    voxelCoord[0] = (x*resolution) - startXinCoordinates;
    voxelCoord[1] = (y*resolution) - startYinCoordinates;
    voxelCoord[2] = (z*resolution) - startZinCoordinates;
    return voxelCoord;
}
std::vector<int> Map::getCageCoordinates(double x,double y, double z){
        std::vector<int> cageCoords;
        int cageX = static_cast<int>((x+startXinCoordinates)/double(resolution));
        int cageY = static_cast<int>((y+startYinCoordinates)/double(resolution));
        int cageZ = static_cast<int>((z+startZinCoordinates)/double(resolution));
        cageCoords.push_back(cageX);
        cageCoords.push_back(cageY);
        cageCoords.push_back(cageZ);
        return cageCoords;
}
void Map::Bresenham(std::vector<int> values,double value){
  int x2 = values[0];
  int y2 = values[1];
  int z2 = values[2];
  int x1 = startXinCages;
  int y1 = startYinCages;
  int z1 = startZinCages;
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int dz = abs(z2 - z1);
  int xs = x2>x1? 1 :-1;
  int ys = y2>y1? 1 :-1;
  int zs = z2>z1? 1 :-1;
 
  // Driving axis is X-axis"
  if (dx >= dy && dx >= dz) {
    int p1 = 2 * dy - dx;
    int p2 = 2 * dz - dx;
    while (x1 != x2) {
      x1 += xs;
      if (p1 >= 0) {
        y1 += ys;
        p1 -= 2 * dx;
      }
      if (p2 >= 0) {
        z1 += zs;
        p2 -= 2 * dx;
      }
      p1 += 2 * dy;
      p2 += 2 * dz;
       if(grid[x1][y1][z1] < 0.1)
         grid[x1][y1][z1] = double(0.1);
       if(grid[x1][y1][z1] > 0.99)
         grid[x1][y1][z1] = double(0.99);  
      
      double L = log(grid[x1][y1][z1]/(1-grid[x1][y1][z1]))+bresCageL;
      double probability = 1.0/double(1.0+exp(-L));
      this->grid[x1][y1][z1] = probability;
    }
 
    // Driving axis is Y-axis"
  }
  else if (dy >= dx && dy >= dz) {
    int p1 = 2 * dx - dy;
    int p2 = 2 * dz - dy;
    while (y1 != y2) {
      y1 += ys;
      if (p1 >= 0) {
        x1 += xs;
        p1 -= 2 * dy;
      }
      if (p2 >= 0) {
        z1 += zs;
        p2 -= 2 * dy;
      }
      p1 += 2 * dx;
      p2 += 2 * dz;
       if(grid[x1][y1][z1] < 0.1)
         grid[x1][y1][z1] = double(0.1);
       if(grid[x1][y1][z1] > 0.99)
         grid[x1][y1][z1] = double(0.99);  

      double L = log(grid[x1][y1][z1]/(1-grid[x1][y1][z1]))+bresCageL;
      double probability = 1.0/double(1.0+exp(-L));
      this->grid[x1][y1][z1] = probability;
      
    }
 
    // Driving axis is Z-axis"
  }
  else {
    int p1 = 2 * dy - dz;
    int p2 = 2 * dx - dz; 
    while (z1 != z2) {
      z1 += zs;
      if (p1 >= 0) {
        y1 += ys;
        p1 -= 2 * dz;
      }
      if (p2 >= 0) {
        x1 += xs; 
        p2 -= 2 * dz;
      }
       if(grid[x1][y1][z1] < 0.1)
         grid[x1][y1][z1] = double(0.1);
       if(grid[x1][y1][z1] > 0.99)
         grid[x1][y1][z1] = double(0.99);  
      
      double L = log(double(grid[x1][y1][z1])/double(1-grid[x1][y1][z1]))+bresCageL;
      double probability = 1.0/double(1.0+exp(-L));
      this->grid[x1][y1][z1] = probability;
      
    }
  }

     if(grid[x2][y2][z2] < 0.1)
         grid[x2][y2][z2] = double(0.1);
     if(grid[x2][y2][z2] > 0.99)
         grid[x2][y2][z2] = double(0.99);  

    double L = log(grid[x2][y2][z2]/double(1.0-(grid[x2][y2][z2])));

    L+=log(value/double(1.0-value));

    double probability = 1.0/double(1.0+exp(-L))  ;
    
    this->grid[x2][y2][z2] = probability;
}


void Map::setValueToMap(double x, double y, double z, float value){ 
        std::vector<int> cageValues = getCageCoordinates(x,y,z);
            if((cageValues[0]>=0 and cageValues[0]<sizeOfMapX)&&(cageValues[1]>=0 && cageValues[1]<sizeOfMapY)&&(cageValues[2]>=0 && cageValues[2]<sizeOfMapZ))
            {
               Bresenham(cageValues,value);    
            }    
}

void Map::clear(){
    for(int x =0; x<sizeOfMapX;x++){
        for(int y=0; y<sizeOfMapY;y++){
            for(int z=0; z<sizeOfMapZ;z++){
                grid[x][y][z] = 0;
            }
        }
    }
}
const vector3d_t& Map::getMap(){
    return grid;
}