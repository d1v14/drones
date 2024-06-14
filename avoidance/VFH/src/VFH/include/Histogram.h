#pragma once
#include "math.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <ros/ros.h>

class Histogram{
public:
    Histogram() = default;
    Histogram(double maxRange,double minRange, int resolution,double distThreshold);
    void updateHistogram(sensor_msgs::LaserScan& scan);
public:
    double                                  maxRange;
    double                                  minRange;
    int                                     resolution;
    double                                  distThreshold;
    double                                  startAngle;
    double                                  endAngle;  
    double                                  sliceSize;
    double                                  shift;
    std::vector<double>                     histRanges;
    std::vector<bool>                       histBinary;
    std::vector<double>                     histSeg;
};