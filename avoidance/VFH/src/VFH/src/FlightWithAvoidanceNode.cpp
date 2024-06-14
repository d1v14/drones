#include 	<ros/ros.h>
#include 	<geometry_msgs/PoseStamped.h>
#include	"FlightCommander.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_controller_node");
	ros::NodeHandle n;
	ros::Rate rate (60);
	FlightCommander controller(n);
	while(ros::ok() && !controller.connectionStatus()){
        ros::spinOnce();
        rate.sleep();
    }

	ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/vehicle/desPose",n);
	controller.arm(true);
	while(ros::ok() && !controller.takeoff()){
		rate.sleep();
		ros::spinOnce();
	}
	while(ros::ok())
	{
		controller.calculateAndSendSetpoint();
		rate.sleep();
		ros::spinOnce();
	}
}