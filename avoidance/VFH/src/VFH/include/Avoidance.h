#pragma once
#include "math.h"
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "Histogram.h"
#include <ros/ros.h>
class Avoidance{
public:
    Avoidance();
    void scanHandler(sensor_msgs::LaserScan& scan);
    std::vector<double> findTrajectory(std::vector<double>& positions);
    int findClosestID(const std::vector<double>& positions);
private:
    Histogram histogram;    
};