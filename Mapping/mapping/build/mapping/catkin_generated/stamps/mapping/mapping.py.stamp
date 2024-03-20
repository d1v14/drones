import rospy
import numpy as np
from threading import Thread

import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg

import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray


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
        rospy.loginfo("%f %f %f",x,y,z)
        self.marker.points.append(point)
        
    def send_update(self):
        self._pub.publish(self.marker_array)

class Grid3D:
    def __init__(self, x_size: float, y_size: float, z_size: float,
                 origin_x: int=50, origin_y: int=50,
                 origin_z: int=3, resolution: float = 1):
        # инициализируем карту 
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        # устанавливаем разрешение карты(размеры вокселей)
        self.cell_size_x = resolution
        self.cell_size_y = resolution
        self.cell_size_z = resolution
        # Устанавливаем точку 0 на карте
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._origin_z = origin_z
        
        self._origin_x_pose = float(self._origin_x * resolution)
        self._origin_y_pose = float(self._origin_y * resolution)
        self._origin_z_pose = float(self._origin_z * resolution)

        # Создаем массив заданной размерности N x M x K
        self.grid = [[[0 for _ in range(z_size)] for _ in range(y_size)] for _ in range(x_size)]

    def _get_cell_coordinates(self, x: float, y: float, z: float):
        # Переводим координаты с плавающей точкой, в порядковый номер ячейки x y z
        cell_x = int((x + self._origin_x_pose) / float(self.cell_size_x))
        cell_y = int((y + self._origin_y_pose) / float(self.cell_size_y))
        cell_z = int((z + self._origin_z_pose) / float(self.cell_size_z))
        return cell_x, cell_y, cell_z

    def set_value(self, x: float, y: float, z: float, value: float):
        # Устанавливаем значение ячейки по заданным координатам
        cell_x, cell_y, cell_z = self._get_cell_coordinates(x, y, z)
        # Проверяем что порядковые номера ячеек не выходят за границы массива, в противном случае
        # игнорируем установку значение. Для усовершенствования можно добавить в таком случае методы
        # для расширения массива или его сдвига(например при использовании циклического буфера)
        if 0 <= cell_x < self.x_size and 0 <= cell_y < self.y_size and 0 <= cell_z < self.z_size:
            self.grid[cell_x][cell_y][cell_z] = value

    def get_value(self, x: float, y: float, z: float):
        # Получаем значение ячейки по координатам
        cell_x, cell_y, cell_z = self._get_cell_coordinates(x, y, z)
        # Проверяем что порядковые номера ячеек не выходят за границы массива
        if 0 <= cell_x < self.x_size and 0 <= cell_y < self.y_size and 0 <= cell_z < self.z_size:
            return self.grid[cell_x][cell_y][cell_z]
        else:
            return None
    
    def __iter__(self):
        # Этот метод возвращает итератор из итерируемого объекта, то есть 
        # нашей карты. 
        self.x_iter = 0
        self.y_iter = 0
        self.z_iter = 0
        # Так как класс сам является итератором то возвращаем
        # self предварительно обнулив индексы для итератора.
        return self

    def __next__(self):
        # Этот метод возвращает следующий элемент из итератора. 
        while True:
            if self.x_iter < self.x_size:
                # Получаем координаты блока в зависимости от точки отсчета(origin)
                # и систем координат.
                x = (self.x_iter * self.cell_size_x) - self._origin_x_pose
                y = (self.y_iter * self.cell_size_y) - self._origin_y_pose
                z = (self.z_iter * self.cell_size_z) - self._origin_z_pose
                value = self.grid[self.x_iter][self.y_iter][self.z_iter]
                
                # Выполняем сдвиг порядкового номера элемента на каждом шаге
                self.z_iter += 1
                if self.z_iter >= self.z_size:
                    self.z_iter = 0
                    self.y_iter += 1
                    if self.y_iter >= self.y_size:
                        self.y_iter = 0
                        self.x_iter += 1
                # Возвращаем координаты блока и значение в нем.
                return x, y, z, value 
            else:
                raise StopIteration



class GridMapping:
    def __init__(self) -> None:
        # Подписываемся на сообщения об облаке точек от камеры глубины
        rospy.Subscriber('/realsense_d435_depth/points', PointCloud2, self.point_cloud_callback)
        # Задаем разрешение карты
        self.map_resolution = 0.5
        self.visual_update_rate = 0.5
        # Создаем нашу карту в виде сетки с заданным разрешением
        self._grid_map = Grid3D(100, 100,  20,
                                resolution=self.map_resolution,
                                origin_x= 50, origin_y= 50, origin_z= 3)
        
        # Создаем объект для визуализации
        self._visualizer = VoxelVisualizer(size=self.map_resolution)
        
        # создаем объект потока для визуализации
        self._mapping = Thread(target=self.mapping_cycle)
        
        # создаем объект буфера, который хранит положение и ориентацию всех систем координат
        # организованных на нашем аппарате, в течении некоторого времени. Эти данные
        # нужны для того чтобы переводить данные различных датчиков в нужные там системы координат
        self.tf_buffer = tf2_ros.Buffer()
        # Подписываемся на данные о положении и ориентации систем координат
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Запускаем поток обновления карты с очень низкой частотой,
        # чтобы не занимать вычислительные ресурсы.
        self._mapping.start()
        
    def mapping_cycle(self):
        rate = rospy.Rate(self.visual_update_rate)
        while (not rospy.is_shutdown()):
            self.send_visualization()
            rate.sleep()
    
    def send_visualization(self):
        # очищаем текущий массив карты
        self._visualizer.clear()
        #  добавляем все точки в массив и отправляем его
        for x, y, z, value in self._grid_map:
            if int(value) == 1: 
                rospy.loginfo("%f %f %f",x,y,z)
                self._visualizer.add_marker(x, y, z)
        self._visualizer.send_update()
    
    def update_map(self, points_map, vehicle_pose):
        # vehicle_pose может быть полезен для получения номеров пустых клеточек
        # Для всех точек выполняем установку клеточки по координатам как занятую.
        for point in points_map:
            # Установка занятости ячейки
            self._grid_map.set_value(point[0], point[1], point[2], 1.0) 
            
    def point_cloud_callback(self, msg):
        # Преобразование облака точек в формат PCL
        cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(list(cloud))
        # Проверка на существование облака точек
        if pcl_cloud is None:
            return
        # Обрезка облака точек по максимальной и минимальной дальности
        # В библиотеке pcl создается специальный экземпляр фильтра.
        clip_filter = pcl_cloud.make_passthrough_filter() 
        # Установка максимального и минимального значения дальности 
        clip_filter.set_filter_limits(0.1, 8.0)
        # Установка оси по которой будет осуществляться фильтрация
        clip_filter.set_filter_field_name("z") 
        # Вызов метода фильтрации
        clipped_cloud = clip_filter.filter()
        
        # Создание фильтра для усреднения облака точек с уменьшением разрешения
        voxel_filter = clipped_cloud.make_voxel_grid_filter()
        # Устанавливаем размер области усреднения
        voxel_filter.set_leaf_size(self.map_resolution, self.map_resolution, self.map_resolution)
        # Выполняем уменьшение разрешение облака точек(можно сравнить с уменьшением разрешения изображения)
        downsampled_points = voxel_filter.filter()
        
        # Выполняем перевод облака точек обратно в формат ros для перевод в систему координат карты
        ros_cloud = pc2.create_cloud_xyz32(msg.header, downsampled_points.to_list())
        # Далее выполняется перевод в систему координат карты
        try:
            # получаем объект трансформации(по сути матрицу поворотов и вектор смещения), для перехода из системы координат камеры глубины
            # в систему координат карты("map"). Обязательно передаем в качестве аргумента метку времени сообщения (msg.header.stamp)
            # в противном случае мы можем получить матрицу и смещения для другого момента времени, не связанного с сообщением, и получим
            # облако точек не в том месте на карте где оно было получено. rospy.Duration(4.0) это время в течении которого сы ждем получение
            # преобразования, в противном случае получаем ошибку.
            trans = self.tf_buffer.lookup_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(4.0))
            # Выполняем переход облака точек между системами координат.
            transformed_cloud_ros = do_transform_cloud(ros_cloud, trans)
            # Получаем положение и ориентацию аппарата в момент получения облака точек
            vehicle_tf = self.tf_buffer.lookup_transform("map", "base_link", msg.header.stamp)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            rospy.logerr(e)
            return
        
        # Выполняем обратное преобразование облака точек формата ros, в формат списка с которым будем работать далее
        points_map  = list(pc2.read_points(transformed_cloud_ros, field_names=("x", "y", "z"), skip_nans=True))
        # Вызываем метод для обновления карты
        self.update_map(points_map, vehicle_tf)

    

if __name__ == "__main__":
    # Инициализируем ноду
    rospy.init_node('simple_mapping')
    # Создаем класс построения карты
    map_builder = GridMapping()
    # Запускаем поток обновления событий ros.
    rospy.spin()
    
