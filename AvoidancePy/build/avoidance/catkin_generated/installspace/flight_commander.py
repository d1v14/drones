import rospy
import numpy as np
import time
from threading import Thread
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import CompanionProcessStatus
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import PositionTarget, State, Trajectory
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition

from planners.path_visualization import PathVisualizer


class FlightCommander:
    def __init__(self) -> None:
        self.current_state = None
        self._pose = None
        self._setpoint = None
        
        self.setpoint_thread = None
        self.stop_thread = False
        self._avoidance_enable = False
        self._origin = None
        self._cur_local_pose = None
        self._prev_local_pose = None
        self._trajectory_prev = None
        self._trajectory_gen_prev = None
        
        self._trajectory = None
        self._generated_trajectory = None
        self._visualizer = PathVisualizer()

        self.init_ros()
        self.init_const()

    def init_const(self):
        # Инициализируем битовые маски для установки целевого состояния ЛА
        self._position_setpoint_mask = PositionTarget.IGNORE_VX \
                                        + PositionTarget.IGNORE_VY \
                                        + PositionTarget.IGNORE_VZ \
                                        + PositionTarget.IGNORE_AFX \
                                        + PositionTarget.IGNORE_AFY \
                                        + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW_RATE
                                        
        self._velocity_setpoint_mask = PositionTarget.IGNORE_PX \
                                        + PositionTarget.IGNORE_PY \
                                        + PositionTarget.IGNORE_PZ \
                                        + PositionTarget.IGNORE_AFX \
                                        + PositionTarget.IGNORE_AFY \
                                        + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW
    
    def init_ros(self) -> None:
        # Создаем объекты для отправки целевых состояний, траекторий ЛА, и состояния бортового компьютера
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.traj_pub = rospy.Publisher('/mavros/trajectory/generated', Trajectory, queue_size=10)
        self._onboard_comp_status_pub = rospy.Publisher('/mavros/companion_process/status', CompanionProcessStatus, queue_size=10)
        
        # Таймер для вызова метода отправки траектории с заданной частотой
        self.traj_timer = rospy.Timer(rospy.Duration(0.1), self.send_trajectory)
        # Таймер для вызова метода отправки траектории с заданной частотой
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self.send_status)
        # Создаем сообщение для отправки состояния системы облета препятствий
        self._onboard_status = CompanionProcessStatus()
        self._onboard_status.component = CompanionProcessStatus.MAV_COMP_ID_OBSTACLE_AVOIDANCE
        
        # Ожидаем получение сообщений и появления сервисов
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/land')
        rospy.wait_for_service('/mavros/set_mode')
        
        # Подписка на состояние ЛА
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        # Подписка на целевую траекторию ЛА
        rospy.Subscriber('/mavros/trajectory/desired', Trajectory, self.trajectory_callback)
        # Подписка на оценку положения ЛА
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
        # Подписка на маршрут полетного задания
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.mission_callback)
        # Подписка на положение точки отсчета локальной СК ЛА
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_pose_callback)
    
    def local_pose_callback(self, msg):
        self._cur_local_pose = msg
        if self._prev_local_pose is not None:
            self._visualizer.update_actual_path(self._prev_local_pose.pose.position,
                                                self._cur_local_pose.pose.position)
            
        self._prev_local_pose = self._cur_local_pose
        
    def home_pose_callback(self, msg):
        if self._origin == None:
            self._origin = msg
        
    def mission_callback(self, msg):
        if self._visualizer is None:
            return
        if self._origin is None:
            return
        # Визуализируем желаемый маршрут ЛА
        self._visualizer.publish_mission_waypoints(msg, self._origin.geo)
    
    def send_status(self, event):
        # Отправка статуса о состоянии системы облёта препятствий ЛА
        self._onboard_status.state = CompanionProcessStatus.MAV_STATE_ACTIVE
        self._onboard_comp_status_pub.publish(self._onboard_status)
        
    def send_trajectory(self, event):
        # Отправляем сгенерированную траекторию в автопилот
        if self._generated_trajectory is None:
            return
        self._visualizer.update_adopted_path(self._cur_local_pose.pose.position,
                                            self._generated_trajectory.point_1.position)
        self.traj_pub.publish(self._generated_trajectory)
        
    def trajectory_callback(self, msg):
        # Получаем и визуализируем желаемую траекторию ЛА от полетного контроллера
        self._trajectory = msg
        if self._trajectory_prev is not None:
            self._visualizer.update_waypoint_path(self._trajectory.point_1.position,
                                                  self._trajectory.point_2.position)
        self._trajectory_prev = self._trajectory
    
    @property
    def desired_trajectory(self):
        return self._trajectory
    
    @property
    def generated_trajectory(self):
        return self._generated_trajectory
    
    @generated_trajectory.setter
    def generated_trajectory(self, traj: Trajectory):
        self._generated_trajectory = traj
        
    def start_setpoint_thread(self):
        self.stop_thread = False
        self.setpoint_thread = Thread(target=self.send_setpoints_loop)
        self.setpoint_thread.start()

    def stop_setpoint_thread(self):
        self.stop_thread = True
        if self.setpoint_thread:
            self.setpoint_thread.join()
            self.setpoint_thread = None

    def send_setpoints_loop(self, target_rate: float=20):
        rate = rospy.Rate(target_rate)  
        while not self.stop_thread and not rospy.is_shutdown():
            if self._setpoint is None:
                continue
            if self.current_state.armed == False:
                self.arm_vehicle
            self.setpoint_pub.publish(self._setpoint)
            rate.sleep()
        self._setpoint = None
    
    def state_callback(self, msg):
        self.current_state = msg
        if msg.mode != "OFFBOARD" and self.setpoint_thread is not None:
            self.stop_setpoint_thread()
    
    @property
    def pose(self):
        return self._cur_local_pose
        
    def land_vehicle(self):
        # Для посадки вызываем сервис с заданными параметрами
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            land_response = land_service(0, 0, 0, 0, 0)
            if land_response.success:
                print("Landing command sent successfully!")
                return True
            else:
                print("Failed to send landing command.")
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
    
    def do_takeoff(self, altitude: float=1.0):
        # Для взлета выполним арм аппарата и зададим ему целевую точку
        if not self.current_state.armed:
            if self.arm_vehicle():
                print("Vehicle armed successfully!")
            else:
                print("Failed to arm the vehicle.")
                return

        self.set_target_position(self._pose.position.pose.x,
                                 self._pose.position.pose.y,
                                 altitude, 0)
        if self.current_state.mode != "OFFBOARD":
            if self.change_mode("OFFBOARD"):
                print("Offboard mode set successfully!")
            else:
                print("Failed to set Offboard mode.")
                return
    
    def arm_vehicle(self, arm: bool=True):
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_response = arm_service(arm)  # True to arm, False to disarm
            return arm_response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
        
    def change_mode(self, mode: str):
        try:
            if mode == "OFFBOARD" and self.setpoint_thread is None:
                self.start_setpoint_thread()
                time.sleep(1)
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(0, mode)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
    def set_offboard_mode(self):
        if self.current_state:
            self.change_mode("OFFBOARD")

    
    def set_velocity_local(self, vx: float, vy: float, vz: float, yaw_rate: float=0):
        setpoint = PositionTarget()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self._set_velocity(setpoint, vx, vy, vz, yaw_rate)


    def set_velocity_body(self, vx: float, vy: float, vz: float, yaw_rate: float=0):
        setpoint = PositionTarget()
        setpoint.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self._set_velocity(setpoint, vx, vy, vz, yaw_rate)

      
    def _set_velocity(self, setpoint: PositionTarget, vx: float,
                      vy: float, vz: float, yaw_rate: float=0):
        setpoint.type_mask = self._velocity_setpoint_mask
        setpoint.velocity.x = vx
        setpoint.velocity.y = vy
        setpoint.velocity.z = vz
        setpoint.yaw_rate = yaw_rate
        self._setpoint = setpoint


    def set_target_position(self, x: float, y: float, z: float, yaw: float=0):
        self.set_offboad_mode()
        setpoint = PositionTarget()
        setpoint.type_mask = self._position_setpoint_mask
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = yaw
        self._setpoint = setpoint


    def go_to_point(self, x :float, y :float, z :float, confidence: float = 0.1):
        distance = confidence * 2
        while np.abs(distance) > confidence:
            self.set_target_position(self, x, y, z)
            distance = np.sqrt((x - self._pose.position.pose.x)**2 \
                            + (y - self._pose.position.pose.y)**2 \
                            + (z - self._pose.position.pose.z)**2)