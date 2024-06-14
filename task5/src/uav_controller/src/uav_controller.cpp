#include "uav_controller.hpp"

UavController::UavController(const ros::NodeHandle &n, const std::string &uavName)
{
	this->n = n;
	this->uavName = uavName;
	rosNodeInit();
	setPointTypeInit();
}

void	UavController::arm(const bool cmd){
	this->setArming.request.value = cmd;
	this->setModeName.request.custom_mode = "OFFBOARD";
		if(this->setModeClient.call(setModeName) && setModeName.response.mode_sent){
			ROS_INFO("OFFBOARD MODE ON");
		}
		if(this->setArmingClient.call(setArming) && setArming.response.success){
				ROS_INFO("UAV ARMED");
			}
		
}
bool UavController::connectionStatus(){
	return this->currentState.connected;
}	

void	UavController::rosNodeInit()
{
	// здесь должна быть инициализация объектов издателя и подписчика для получения информации из необходимых топиков
	// пример инициализации подписки на топик желаемого положения ЛА

	destVelocityPub = this->n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	// local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	stateSub = this->n.subscribe<mavros_msgs::State>("/mavros/state",10,&UavController::uavStateCallback,this);
	localPositionSub = this->n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&UavController::localPositionCallback,this);
	destPosSub = this->n.subscribe<geometry_msgs::PoseStamped>("/vehicle/desPose", 1, &UavController::desiredPositionCallback,this);

	setArmingClient = this->n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	setModeClient = this->n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}


void	UavController::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    this->currentState = *msg;
}


void    UavController::localPositionCallback(const geometry_msgs::PoseStamped localPose)
{

	this->currentPoseLocal = localPose;
}


void    UavController::desiredPositionCallback(const geometry_msgs::PoseStamped destPoint)
{
	this->destPoint = destPoint;
}


void	UavController::calculateAndSendSetpoint()
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
	double errorx,errory,errorz;
	errorx = this->destPoint.pose.position.x - this->currentPoseLocal.pose.position.x;
	errory = this->destPoint.pose.position.y - this->currentPoseLocal.pose.position.y;
	errorz = this->destPoint.pose.position.z - this->currentPoseLocal.pose.position.z;

	// setPoint.velocity.x = 0.1*errorx;
	// setPoint.velocity.y = 0.1*errory;
	setPoint.velocity.x = 1*errorx;
	setPoint.velocity.y = 1*errory;
	setPoint.velocity.z = 1*errorz;
	destVelocityPub.publish(setPoint);
	
}

void 	UavController::setPointTypeInit()
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