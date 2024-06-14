import numpy as np
import heapq
from mapping.simple_grid_map import GridMap3D
from .planer_base import Node, PlanerError, APathPlaner

NEIGHBORS_SHIFT_2D = [(0, -1), (0, 1),
                      (-1, 0), (1, 0),
                      (-1, -1), (-1, 1),
                      (1, -1), (1, 1)]

class AStarPlanner(APathPlaner):
    
    def plan_path(self, start_point: list, target_point: list, map: GridMap3D) -> list:
        # Выполняем проверку на наличие целевой и стартовой точек в карте
        start_idx = map.get_cell_coordinates(start_point[0], start_point[1], start_point[2])
        target_idx = map.get_cell_coordinates(target_point[0], target_point[1], target_point[2])
        if not map.is_index_valid(start_idx) or not map.is_index_valid(target_idx):
            raise PlanerError("Points not found in grid map")
        # Соз
        start_node = Node(start_idx, None)
        target_node = Node(target_idx, None)
        open_set = [] 
        closed_set = set()
        # open_set.append(start_node)
        heapq.heappush(open_set, start_node)
        # closedList = {} # elements which vere visited
        
        while len(open_set) > 0:
            # get current node
            current_node = heapq.heappop(open_set)
            closed_set.add(current_node.pose)
            
            nodes = self._get_neighbors_nodes(current_node, closed_set, open_set, map)
            for node in nodes:
                # update node heuristic 
                self._heuristic(node, target_node)
                heapq.heappush(open_set, node)
                
            # check that target is reached
            if self._check_target_reached(current_node, target_node):
                return self._get_node_path(current_node, map)
            
        return []
                              
    def _get_neighbors_nodes(self, current_node: Node,
                             closed_set: set, open_set: list,
                             map: GridMap3D) -> list:
        # generate new nodes
        neighbors_nodes = []
        for i in range(3):
            for shift in NEIGHBORS_SHIFT_2D:
                node_pose = (current_node.pose[0] + shift[0],
                            current_node.pose[1] + shift[1],
                            current_node.pose[2] + i - 1)
                
                # Проверяем что индекс не выходит за границе массива карты
                if not map.is_index_valid(node_pose):
                    continue
                
                # Проверяем что ячейка не занята
                if np.isclose(1.0, map.get_value_by_indexes(node_pose[0], node_pose[1], node_pose[2])):
                    continue
                
                # Проверяем что элемент еще не добавлен в список
                if self._check_node_in_list(node_pose, open_set):
                    continue
                
                # Проверяем что элемент еще не посещен
                if node_pose in closed_set:
                    continue
                
                # Добавляем ячейку в список соседей
                neighbors_nodes.append(Node(node_pose, current_node))
                
        # Возвращаем полученный список соседних узлов
        return neighbors_nodes
    
    def _check_node_in_list(self, node_pose: tuple, nodes: list) -> bool:
        for node in nodes:
            if node.pose == node_pose:
                return True
        return False
              
    def _check_target_reached(self, current_node: Node, target_node: Node) -> bool:
        for i in range(3):
            if current_node.pose[i] - target_node.pose[i] != 0:
                    return False
        return True
    

    def _heuristic(self, cur_node: Node, target_node: Node) -> None:
        cur_node.g = cur_node.parent.g + 1
        # manhetan norm
        h = (target_node.pose[0]-cur_node.pose[0])**2 + \
            (target_node.pose[1]-cur_node.pose[1])**2 + \
            (target_node.pose[2]-cur_node.pose[2])**2
        cur_node.cost = h + cur_node.g
        
    
    def _get_node_path(self, node: Node, map: GridMap3D) -> list:
        path = []
        while node is not None:
            path.append(map.get_cell_pose_by_ids(node.pose))
            node = node.parent
        return path