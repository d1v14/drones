import numpy as np
from mapping.simple_grid_map import GridMap3D
from .planer_base import Node, PlanerError, APathPlaner

class RRTStarPlaner(APathPlaner):
    def __init__(self, max_iterations: int, 
                 goal_sample_rate: int,
                 expand_distance: float,
                 connect_dist: float,
                 vehicle_radius: float) -> None:
        self._max_iterations = max_iterations
        self._goal_sample_rate = goal_sample_rate
        self._start_node = None
        self._end_node = None
        self._node_list = None
        self._expand_distance = expand_distance
        self._connect_dist = connect_dist
        self._vehicle_radius = vehicle_radius
    
    def get_random_node(self, size: tuple) -> Node:
        X = np.random.uniform(0, size[0])
        Y = np.random.uniform(0, size[1])
        Z = np.random.uniform(0, size[2])
        return Node((X, Y, Z), None)
    
    def calculate_distance(self, first_pose, second_pose) -> float:
        return np.sqrt((first_pose[0] - second_pose[0])**2,
                       (first_pose[1] - second_pose[1])**2,
                       (first_pose[2] - second_pose[2])**2) 
        
    
    def get_nearest_node_id(self, cur_node: Node) -> int:
        distances = [self.calculate_distance(cur_node, node) \
                     for node in self._node_list]
        return distances.index(min(distances))
    
    def find_nearest_nodes(self, cur_node: Node):
        node_count = len(self.node_list) + 1
        #?????
        r = self.connect_dist * np.sqrt(np.log(node_count) / node_count)
        distances =  [self.calculate_distance(cur_node, node) \
                     for node in self._node_list]
        near_ids = [distances.index(i) for i in distances if i <= r**2]
        return distances
    
    def get_point_in_line(first_point: tuple, second_point: tuple, distance: float):
       first_point = np.array(first_point)
       second_point = np.array(second_point)
       direction = second_point - first_point
       direction_unit_vector =  direction / np.linalg.norm(direction)
       
       return (first_point + (direction_unit_vector * distance))
       
    def steer(self, node: Node, pose: tuple) -> Node:
        # Рассчитаем расстояние между узлом и заданной случайной точкой.
        distance = self.calculate_distance(node.pose, pose)
        # Проверяем что расстояние между узлом и точкой не превышает
        # пороговое значение. Расстояние между узлами не должно быть слишком большим.
        if distance > self._expand_distance: distance = self._expand_distance
        # Получим положение нового узла дерева. Для этого перепроектируем полученную 
        # дистанцию от узла в направлении случайной точки.
        new_node_pose = self.get_point_in_line(node.pose, pose, distance)
        # Вернем новый узел.
        return Node((new_node_pose[0], new_node_pose[1], new_node_pose[2]), node)
    
    def plan_path(self, start_point: list, target_point: list, map: GridMap3D) -> list:
        # Перевод точек в индексы карты
        start_idx = map.get_cell_coordinates(start_point[0], start_point[1], start_point[2])
        target_idx = map.get_cell_coordinates(target_point[0], target_point[1], target_point[2])
        self._start_node = Node(start_idx, None)
        self._end_node = Node(target_idx, None)
        self._node_list = [self._start_node]
        
        path_found = False
        
        # До тех пор пока не достигнем максимального количества итераций
        # будем выполнять расширение дерева
        for i in range(self._max_iterations):
            # Получаем случайную точку в пределах карты 
            random_node = self.get_random_node(map.size)
            # Находим индекс ближайшего узла для случайной точки
            nearest_ind = self.get_nearest_node_id(self._node_list, random_node)
            # Выполняем расширение дерева, строим маршрут от ближайшего узла до случайной
            # точки.
            # При этом расстояние ограничено чтобы избежать слишком больших
            # шагов.
            new_node = self.steer(self._node_list[nearest_ind], random_node)
            # Проверяем что точка не находится внутри препятствия.
            if map.check_collisions_between_points(self._node_list[nearest_ind].pose,
                                                   new_node.pose):
                continue
            
            # Ищем ближайший узел с наилучшей стоимостью
            nearest_ids = self.find_nearest_nodes(new_node)
            node_with_new_parent = self.set_parent(new_node, nearest_ids)
            if node_with_new_parent:
                # Выполняем оптимизацию маршрута. Ищем более короткие пути для каждой точки 
                # которые могли появиться в ходе работы алгоритма и появления новых узлов.
                self.rewire(node_with_new_parent, nearest_ids)
                self._node_list.append(node_with_new_parent)
            else:
                self._node_list.append(new_node)
            
            # Проверяем попадает ли новый узел в целевую точку.
            if self._end_node == new_node:
                path_found == True
        
        if (path_found):
            last_index = self.search_best_goal_node()
            return self._get_path(last_index)
    
        return []
    
    def _get_path(self, node: Node, map: GridMap3D) -> list:
        path = []
        while node is not None:
            path.append(map.get_cell_pose_by_ids(node.pose))
            node = node.parent
        return path 

    
    def set_parent(self, new_node, nearest_ids, map: GridMap3D):
        if not nearest_ids:
            return None
        
        costs = []
        for i in nearest_ids:
            near_node = self._node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and not map.check_collisions_in_area(t_node, 
                                                           self._vehicle_radius):
                costs.append(0)
            else:
                costs.append(float("inf"))
        
        min_cost = min(costs)
        if min_cost == float("inf"):
            return None
        
        min_ind = nearest_ids[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost
        
    def find_best_path(self) -> list:
        return []
    
    def rewire(self, node: Node, near_ids) -> None:
        pass