import numpy as np
import matplotlib.pyplot as plt

import rospy
import geometry_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from .histogram2d import Histogram2d


class HistogramVisualizer:
    def __init__(self) -> None:
        # Создадим обьект издателя для визуализации гистограммы
        self._viz_pub = rospy.Publisher('/vfh_visalize', Image, queue_size=1)
        
        self._bridge = CvBridge()

        self.fig = plt.figure()
        self.ax = self.fig.subplots(nrows=2, ncols=2)
        self.fig.delaxes(self.ax[1][0])
        self.fig.delaxes(self.ax[0][0])
        self.ax[0][0] = self.fig.add_subplot(2, 2, 1, projection='polar')
        self.ax[1][1].grid()
        self.ax[0][1].grid()
    
    def update_histogram_image(self, map: Histogram2d):
        image = self._create_histogram_image(map)
        ros_img = self._bridge.cv2_to_imgmsg(image, "rgb8")
        self._viz_pub.publish(ros_img)
    
    def _create_histogram_image(self, map: Histogram2d):
        self.ax[0][0].clear()
        self.ax[0][1].clear()
        self.ax[1][1].clear()
        # Polar plot on the top-left
        self.ax[0][0].bar(map.segment_pose, map.range_hist,
                          width=map.segment_resolution)
        self.ax[0][0].set_theta_offset(np.pi/2)
        
        # First plot on the top-right
        self.ax[0][1].stairs(map.range_hist, fill=True)
        self.ax[0][1].axhline(y=map.dist_trashhold, color='r', linestyle='-')

        # Second plot on the bottom-right
        self.ax[1][1].stairs(map.bin_hist.astype(int), fill=True)
        self.ax[1][1].stairs(map.bin_hist.__invert__().astype(int), hatch='//')

        # Adjust layout for better appearance
        plt.tight_layout()

        # Render the figure to an image
        self.fig.canvas.draw() 
        image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        return image

class VoxelVisualizer:
    def __init__(self, r:float = 0, g:float = 1.0, b :float=0,
                 a :float=1, size: float = 1):
        # Создание публикатора для отправки сообщений типа MarkerArray на топик 'voxel_marker_array'
        self._pub = rospy.Publisher('voxel_marker_array', MarkerArray, queue_size=1)

        # Инициализация массива маркеров
        self.marker_array = MarkerArray()

        # Создание маркера для визуализации вокселей
        self.marker = Marker()
        self.marker.header.frame_id = "map"  # Используемая карта координат
        self.marker.id = 0  # Уникальный идентификатор маркера
        self.marker.type = Marker.CUBE_LIST  # Тип маркера - массив кубиков (вокселей)
        self.marker.action = Marker.ADD  # Действие - добавить маркер

        # Установка ориентации маркера в формате кватерниона
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        
        # Установка размера вокселя
        self.marker.scale.x = size
        self.marker.scale.y = size
        self.marker.scale.z = size

        # Установка цвета вокселя (зеленый с некоторой прозрачностью)
        self.marker.color.r = r
        self.marker.color.g = g
        self.marker.color.b = b
        self.marker.color.a = a
        
        self.marker_array.markers.append(self.marker)
    
    def clear(self):
        # Очистка массива точек
        self.marker.points.clear()
    
    def add_marker(self, x :float, y :float, z :float):
        point = geometry_msgs.msg.Point()
        point.x = x
        point.y = y
        point.z = z
        self.marker.points.append(point)
        
    def send_update(self):
        self._pub.publish(self.marker_array)