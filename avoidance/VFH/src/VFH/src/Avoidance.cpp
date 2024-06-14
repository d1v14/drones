#include "Avoidance.h"
Avoidance::Avoidance(){
    this->histogram = Histogram(15,0.4,8,5);

}
void Avoidance::scanHandler(sensor_msgs::LaserScan& scan){
    histogram.updateHistogram(scan);
}
std::vector<double> Avoidance::findTrajectory(std::vector<double>& points){
    double curX = points[0];
    double curY = points[1];
    double curZ = points[2];
    double destX = points[3];
    double destY = points[4];
    double destZ = points[5];
    int64_t start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if(abs(destX - curX ) <=0.5 && abs(destY - curY ) <=0.5){
        return std::move(std::vector<double>{destZ});
    }
    int freeCages =0;
    int closedCages = 0;
    for(int i = 0 ; i<histogram.histBinary.size();i++)
        if(histogram.histBinary[i] == false)
            freeCages++;
        else
            closedCages++;
    if(closedCages==histogram.histBinary.size())
    {
        return std::move(std::vector<double>{destZ});
    }
    int id = findClosestID(points);
    double targetX = histogram.distThreshold * sin(histogram.histSeg[id] + histogram.shift);
    double targetY = histogram.distThreshold * -cos(histogram.histSeg[id] + histogram.shift);
    targetX += curX;
    targetY += curY;
	int64_t end = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    ROS_INFO("%ld",end-start);
    return std::move(std::vector<double>{targetX,targetY,curZ});

    

}

int Avoidance::findClosestID(const std::vector<double>& points){
    double curX = points[0];
    double curY = points[1];
    double curZ = points[2];
    double destX = points[3];
    double destY = points[4];
    double destZ = points[5];
    double trajectoryAngle = atan2(points[3]-points[0],-(points[4]-points[1]));
    if(trajectoryAngle<0){
        trajectoryAngle += 2*M_PI;
    }
    double bestScore =  0;
    int closestDiraction=-1;
    double trajectoryAngleOnHist;
    for (int i = 0 ; i< histogram.histBinary.size();i++){
            if(histogram.histBinary[i] != true)
            {
                double diff = abs(histogram.histSeg[i] - trajectoryAngle);
            
                if (closestDiraction == -1){
                    closestDiraction = i;
                    bestScore = diff;
                }
                else{
                    if(diff < bestScore){
                    closestDiraction = i;
                    bestScore = diff;
                    }
                } 
            }
    }
    return closestDiraction;
}