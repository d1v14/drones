#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <thread>
#include <tf/tf.h>
#include "Histogram.h"
#include "Avoidance.h"
// Обьявим класс для контроллера БЛА
// этот класс включает методы для управления аппаратом 
// 
class FlightCommander
{
	public:
		ros::NodeHandle		n;
		ros::Time						lastRequest;
		FlightCommander(const ros::NodeHandle &n);

		/**
		 * @brief Метод переводит аппарат в состояние arm/disarm
		 * Состояние arm - аппарат готов к движению при получении комманд
		 * управления начинает движение
		 * 
		 * Состояние disarm - аппарат не готов к движению при поступлении
		 * комманд управления не начинает движение
		 * 
		 * @param cmd - смена состояния
		 * True - перевод аппарата в состояние arm
		 * False - перевод аппарата в состояние disarm
		 */
		void							arm(const bool cmd);
		/**
		 * @brief метод производит рассчет желаемых управляющих воздействий
		 *  и пересылает сообщение типа mavros_msgs::PositionTarget
		 * в топик <имя_ла>/setpoint_raw/local (как правило mavros/setpoint_raw/local)
		 */
		void    						calculateAndSendSetpoint();
		bool							connectionStatus();
		int							takeoff();
	private:
		std::string 					uavName;
		mavros_msgs::PositionTarget		setPoint; // объект сообщения для задающего воздействия
		mavros_msgs::State				currentState; // объект сообщения о состоянии аппарата
		geometry_msgs::PoseStamped		currentPoseLocal; // объект сообщения о положении и ориентации
		geometry_msgs::PoseStamped		destPoint;

		ros::Publisher					setPointPub;
		ros::Publisher					destVelocityPub;
		
		ros::Subscriber					localPositionSub;
		ros::Subscriber					stateSub;
		ros::Subscriber					destPosSub;
		ros::Subscriber					lidarSub;
		
		ros::ServiceClient				setArmingClient;
		ros::ServiceClient				setModeClient;
		mavros_msgs::CommandBool 		setArming;
		mavros_msgs::SetMode 			setModeName;
		
		Avoidance						AvoidanceSystem;
		void							rosNodeInit();
		void 							setPointTypeInit();				
		void							uavStateCallback(const mavros_msgs::State::ConstPtr	&msg);
		void    						desiredPositionCallback(const geometry_msgs::PoseStamped desPose);
		void							localPositionCallback(const geometry_msgs::PoseStamped localPose);
		void							lidarCallback(sensor_msgs::LaserScan scan);;



};
