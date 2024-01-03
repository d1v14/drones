#include 	<ros/ros.h>
#include 	<geometry_msgs/PoseStamped.h>
#include	<uav_controller.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_controller_node");
	ros::NodeHandle n;
	geometry_msgs::PoseStamped		desPose;
	ros::Rate 				rate(50);
	ros::NodeHandle &s = n;
	std::string name("Drone");
	UavController controller(s,name);
	const bool arming = true;	
	while(ros::ok() && !controller.connectionStatus()){
        ros::spinOnce();
        rate.sleep();
    }
	controller.arm(arming);	
	while(ros::ok())
	{
		controller.calculateAndSendSetpoint();
		ros::spinOnce();
		rate.sleep();
	}

}