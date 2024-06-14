#include "Histogram.h"

Histogram::Histogram(double maxRange,double minRange, int resolution,double distThreshold){
    this->minRange = minRange;
    this->maxRange = maxRange;
    this->resolution = resolution;
    this->distThreshold = distThreshold;
    this->startAngle = 0;
    this->endAngle = 2* M_PI;
    this->shift = (this->endAngle - this->startAngle)/resolution/2;
    for(int i=0; i<resolution;i++){
        histRanges.push_back(0);
        histBinary.push_back(false);
        histSeg.push_back(0 + i * (2*M_PI/resolution));
    }
    
}
void Histogram::updateHistogram(sensor_msgs::LaserScan& scan){
    std::vector<double> tempHistRanges(this->resolution,0);
    std::vector<bool> tempBinHistRanges(this->resolution,false);
    for(int i =0; i<scan.ranges.size();i++){
        int ind = (i*scan.angle_increment) /((scan.angle_max - scan.angle_min)/this->resolution);
        if(isinf(scan.ranges[i]))
        {}
        else
            {
                if(scan.ranges[i] < tempHistRanges[ind] || tempHistRanges[ind] == 0.0)
                   tempHistRanges[ind] = scan.ranges[i];
            }
    }
    for(int i=0; i< tempHistRanges.size();i++){
        if(tempHistRanges[i] < this->distThreshold && tempHistRanges[i] > scan.range_min)
            tempBinHistRanges[i] = true;
    }
    
    histBinary = tempBinHistRanges;
    histRanges = tempHistRanges;
}


