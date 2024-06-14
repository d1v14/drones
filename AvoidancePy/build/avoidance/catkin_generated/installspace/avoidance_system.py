import time
from threading import Thread
import numpy as np

import rospy
import tf
import tf2_ros
import tf2_py as tf2

from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import Trajectory


from avoidance.flight_commander import FlightCommander
from planners.planer_base import PathPlannerWrapper
from planners.vfh2d_planner import VFH2d
from mapping.histogram2d import Histogram2d
from mapping.map_visualizer import HistogramVisualizer
            
class AvoidanceSystem:
    def __init__(self, flight_controller: FlightCommander) -> None:
        # Получаем имя топика дальнометрического датчика
        self._range_topic_name = rospy.get_param('~topic_name', '/scan')
        # Получаем параметры рпзрешения гистограммы и порогового значения дальности
        self._histogram_resolution = rospy.get_param('~hist_resolution', 8)
        self._histogram_treshhold = rospy.get_param('~dist_trash', 7.0)
        
        # Создадим подписку на данные дальномера
        rospy.Subscriber(self._range_topic_name, LaserScan, self.range_sensor_cb, queue_size=1)
        
        # Добавим поле объекта управления аппарата
        self._flight_controller = flight_controller
        
        # Добавим метод для инициализации гистограммы
        # он будет исполбзоваться для обработки первого сообщения
        # для инициализации гистограммы
        self.range_cb = self.range_init
        
        # Создадим планировщик маршрута, в нашем случае это VFH2d
        self._planer = PathPlannerWrapper(VFH2d())
        
        # Создадим экземпляр карты, в данном случае  это гистограмма для VFH2d
        self._map = Histogram2d()
        self._histogram_visualizer = HistogramVisualizer()
        
        self._generated_trajectory = Trajectory()
        
        # Создаем объект буфера, который хранит положение и ориентацию всех систем координат
        # организованных на нашем аппарате, в течении некоторого времени. Эти данные
        # нужны для того чтобы переводить данные различных датчиков в нужные там системы координат
        self._tf_buffer = tf2_ros.Buffer()
        # Подписываемся на данные о положении и ориентации систем координат
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        
        # Создаем таймер для визуализации
        self._hist_vis_tm = rospy.Timer(rospy.Duration(1), self._vizualization_tm)
        
    def range_init(self, msg: LaserScan):
        # Инициалихируем гистограмму на основе данных от лидара
        # При этом необходимо убедиться что минимальное значение измерений
        # больше 0, близкие к 0 значения будут восприниматься как отсутствие препятствий
        # Это связано с особенностями работы дальномеров, где в случае невозможности измерения дальностей
        # может приходить нулевое значение
        self._map.init_histogram(resolution=self._histogram_resolution,
                                max_range=msg.range_max,
                                min_range=msg.range_min,
                                start_angle=msg.angle_min,
                                end_angle=msg.angle_max,
                                dist_trash=self._histogram_treshhold)
        # Устанавливаем новый метод для обработки сообщений,
        # в который будет попадать сообщение
        self.range_cb = self.update_avoidance
        
    def range_sensor_cb(self, msg: LaserScan) -> None:
        # Отправляем сообщение в функцию обратного вызова
        self.range_cb(msg)
    
    def shift_laserscan_ranges(self, scan: LaserScan, shift_angle_rad: float):
        # Calculate the number of elements to shift
        num_elements_to_shift = int(round(shift_angle_rad / scan.angle_increment))
        # Shift the ranges array
        shifted_ranges = scan.ranges[-num_elements_to_shift:] + scan.ranges[:-num_elements_to_shift]
        return shifted_ranges
    
    def update_avoidance(self, msg: LaserScan) -> None:
        # Основной метод для обработки данных датчика и формирования траектории для облета препятствий
        # Convert to Euler angles
        if self._flight_controller.pose is None or \
            self._flight_controller.desired_trajectory is None:
            return
        q = self._flight_controller.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        shifted_ranges = self.shift_laserscan_ranges(msg, yaw)
        
        # Обновим гистограмму, при этом нужно учитывать чтобы измерения 
        # датчика были перепроектированы в неподвижную систему координат
        # в которой задаются целевые точки
        self._map.update_histogram(shifted_ranges)
        # Получаем целевую и текущую точки
        desired_trajectory = self._flight_controller.desired_trajectory
        target_pose = desired_trajectory.point_2.position
        current_pose = self._flight_controller.pose.pose.position
        # Заполняем желаемое и текущее положение как списки
        current_pose = [current_pose.x, current_pose.y, current_pose.z]
        target_pose = [target_pose.x, target_pose.y, target_pose.z]
        # Обновляем маршрут
        path = self._planer.find_path(current_pose, target_pose ,self._map)
        # Проверяем что система создала маршрут
        if len(path) == 0: # В нашем случае пустой маршрут означает что путь свободен
            # Отправляем обратно целевую точку 
            self._flight_controller.generated_trajectory = desired_trajectory
            return 
        # Иначе отправляем новый маршрут
        self._generated_trajectory = desired_trajectory
        self._generated_trajectory.header.stamp = rospy.Time().now()
        self._generated_trajectory.point_1.position.x = path[0][0]
        self._generated_trajectory.point_1.position.y = path[0][1]
        self._generated_trajectory.point_1.position.z = path[0][2]
        self._flight_controller.generated_trajectory = self._generated_trajectory
    
    def _vizualization_tm(self, event) -> None:
        # Ожидаем инициализации гистограммы
        if self._map.is_init == False:
            return
        # Обновляем изображение с визуализацией гистограммы
        self._histogram_visualizer.update_histogram_image(self._map)
        