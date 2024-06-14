#include "FlightCommander.hpp"

FlightCommander::FlightCommander(const ros::NodeHandle &n)
{
	this->n = n;
	rosNodeInit();
	setPointTypeInit();
}

void	FlightCommander::arm(const bool cmd){
	this->setArming.request.value = cmd;
	this->setModeName.request.custom_mode = "OFFBOARD";
		if(this->setModeClient.call(setModeName) && setModeName.response.mode_sent){
			ROS_INFO("OFFBOARD MODE ON");
		}
		if(this->setArmingClient.call(setArming) && setArming.response.success){
				ROS_INFO("UAV ARMED");
			}

}
bool FlightCommander::connectionStatus(){
	return this->currentState.connected;
}

void	FlightCommander::rosNodeInit()
{
	destVelocityPub = this->n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	stateSub = this->n.subscribe<mavros_msgs::State>("/mavros/state",10,&FlightCommander::uavStateCallback,this);
	localPositionSub = this->n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&FlightCommander::localPositionCallback,this);
	destPosSub = this->n.subscribe<geometry_msgs::PoseStamped>("/vehicle/desPose", 1, &FlightCommander::desiredPositionCallback,this);
	setArmingClient = this->n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	setModeClient = this->n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	lidarSub = this->n.subscribe<sensor_msgs::LaserScan>("/scan",10,&FlightCommander::lidarCallback,this);
}

void FlightCommander::lidarCallback(sensor_msgs::LaserScan msg){
	AvoidanceSystem.scanHandler(msg);	
}
void	FlightCommander::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    this->currentState = *msg;

}


void    FlightCommander::localPositionCallback(const geometry_msgs::PoseStamped localPose)
{

	this->currentPoseLocal = localPose;
}


void    FlightCommander::desiredPositionCallback(const geometry_msgs::PoseStamped destPoint)
{
	this->destPoint = destPoint;
}

int FlightCommander::takeoff(){
	double targetheight = 5;
	double errorz;
	errorz = targetheight - this->currentPoseLocal.pose.position.z;
	if(errorz > 0.1){
		setPoint.velocity.z = 1*errorz;
		setPoint.velocity.x = 0;
		setPoint.velocity.y = 0;
		destVelocityPub.publish(setPoint);
		return 0;
	}
	else{
		return 1;
	}

}
void	FlightCommander::calculateAndSendSetpoint()
{

	// здесь необходимо выполнить рассчет желаемой линейной скорости ЛА
	// можно пользоваться алгоритмами систем управления которые мы изучили ранее (например П ПД или ПИД регуляторами)
	// destPoint.pose.position.x = 0; // вместо заданного значения должен быть рассчет скорости
	// destPoint.pose.position.y = 0; // вместо заданного значения должен быть рассчет скорости
	// // при публикации такой скорости аппарат будет лететь вдоль оси Z
	// // стартовой СК(по направлению вверх) со скоростью 1 м/сек
	// destPoint.pose.position.z = 1; // вместо заданного значения должен быть рассчет скорости
	// // здесь необходимо выполнить рассчет желаемой угловой скорости ЛА
	// // setPoint.yaw_rate = 0; // вместо заданного значения должен быть рассчет угловой скорости
	// // отправка 
	std::vector<double> points{currentPoseLocal.pose.position.x,currentPoseLocal.pose.position.y,currentPoseLocal.pose.position.z,destPoint.pose.position.x,destPoint.pose.position.y,destPoint.pose.position.z};
	std::vector<double> trajectory = AvoidanceSystem.findTrajectory(points);
    
	if(trajectory.size() > 1){
		double errorx,errory,errorz;
		errorx = trajectory[0] - this->currentPoseLocal.pose.position.x;
		errory = trajectory[1] - this->currentPoseLocal.pose.position.y;
		errorz = trajectory[2] - this->currentPoseLocal.pose.position.z;
		setPoint.velocity.x = 0.5*errorx;
		setPoint.velocity.y = 0.5*errory;
		setPoint.velocity.z = 0.5*errorz;
	}
	else{
		double errorz;
		errorz = trajectory[0] - this->currentPoseLocal.pose.position.z;
		setPoint.velocity.x = 0;
		setPoint.velocity.y = 0;
		setPoint.velocity.z = 0.1*errorz;
	}
	destVelocityPub.publish(setPoint);
	
}

void 	FlightCommander::setPointTypeInit()
{
	// задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
	// приведенная ниже конфигурация соответствует управлению линейной скоростью ЛА
	// и угловой скоростью аппарата в канале рыскания(yaw)
	uint16_t setpointTypeMask =	mavros_msgs::PositionTarget::IGNORE_PX  +
									mavros_msgs::PositionTarget::IGNORE_PY  +
									mavros_msgs::PositionTarget::IGNORE_PZ  +
									mavros_msgs::PositionTarget::IGNORE_AFX +
									mavros_msgs::PositionTarget::IGNORE_AFY +
									mavros_msgs::PositionTarget::IGNORE_AFZ +
									mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	// при помощи конфигурации вышеприведенным образом переменной setpointTypeMask
	// можно настроить управление аппаратом посредством передачи(положения аппарата и углового положения в канале рыскания, )

	// конфигурация системы координат в соответствии с которой задаются параметры управления ЛА
	// при setpointCoordinateFrame = 1 управление происходит в неподвижной СК (локальная неподвижная СК инициализируемая при работе навигационной системы)
	// при импользовании ГНСС или optical flow является стартовой, при использовании других НС начало координат соответствует таковому у выбранной
	//  навигационной системы(например оси выходят из центра реперного маркера).
	// setpointCoordinateFrame = 8 соответствует управлению аппаратом в связных нормальных осях (подвижная СК центр которой находится в центре масс ЛА)
	// в действительности без какой либо настройки, совпадает с системой координат инерциальной навигационной системы.
	unsigned int setpointCoordinateFrame = 1;

	// присваевам наши параметры задающего воздействия полям класса нашего сообщения
	setPoint.type_mask = setpointTypeMask;
	setPoint.coordinate_frame = setpointCoordinateFrame;

}