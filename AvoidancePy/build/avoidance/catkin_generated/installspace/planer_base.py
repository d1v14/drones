from abc import ABC, abstractmethod


class PlanerError(Exception):
    def __init__(self, message="planer error"):
        self.message = message
        super().__init__(self.message)

class Node:
    def __init__(self, pose, parent,
                 cost: int=0, g: int=0):
        self._pose = pose
        self._parent = parent
        self._cost = cost
        self._g = g
        
    @property
    def parent(self):
        return self._parent
        
    @property
    def pose(self):
        return self._pose
    
    @property
    def g(self):
        return self._g
    
    @g.setter
    def g(self, val: int):
        self._g  = val
    
    
    def __lt__(self, other):
        # Define comparison based on the total cost (f = g + h)
        return (self.cost) < (other.cost)

class APathPlaner(ABC):
    def find_path(self, current_pose: list, target_pose: list, map) -> list:
        # Проверяем что целевое и текущее положение имеет корректную размерность
        if len(current_pose) != 3 or len(target_pose) != 3:
            raise PlanerError("Not valid target or current pose")
        return self.plan_path(current_pose, target_pose, map)
    
    @abstractmethod
    def plan_path(current_pose: list, target_pose: list, map) -> list:
        pass
        
    
class PathPlannerWrapper:
    def __init__(self, planner: APathPlaner) -> None:
        self._planner = planner
        
    def find_path(self, current_pose: list, target_pose: list, map) -> list:
        return self._planner.find_path(current_pose, target_pose, map)