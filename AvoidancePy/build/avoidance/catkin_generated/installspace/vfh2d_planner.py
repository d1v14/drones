import numpy as np
from .planer_base import APathPlaner
from mapping.histogram2d import Histogram2d
   

class VFH2d(APathPlaner):
    def __init__(self, direction_treshold: float = 0.1) -> None:
        # Порогавая величина смены направления по гистограмме.
        self._direction_treshold = direction_treshold
        super().__init__()
    
    def plan_path(self, current_pose: list, target_pose: list, map: Histogram2d):
        # Проверяем что целевое и текущее положение нахожятся далеко
        # иначе вернем желаемое положение, в случае если оно находитя рядом(dist_trashhold)
        if np.isclose(current_pose[0], target_pose[0], atol=map.dist_trashhold) and \
            np.isclose(current_pose[1], target_pose[1], atol=map.dist_trashhold) :
            return [target_pose]
        # Проверяем что гистограмма не пустая
        if map.range_hist is None or map.bin_hist is None:
            return []
        
        # Проверим является ли хотя бы один из сегментов гистограммы занятым.
        # В случае если все сегменты свободны вернем пустой список,
        # Таким образом будем считать что препятствия не обнаружены.
        if not np.any(map.bin_hist==True):
            return []
        # Проверим ситуацию, когда все сегменты заняты, в этом случае вернем текущую
        # точку, чтобы аппарат остановился и избежал столкновения.
        elif not np.any(map.bin_hist==False):
            return [current_pose]
        
        # Если хотя бы один сегмент гистограммы свободен, выполним поиск наилучшего сегмента.
       
        closest_direction_id = self._find_closest_direction_id(current_pose, target_pose, map)
        # Вернем точку, ближайшую по направлению к свободному сегменту, с учетом отступа.
        target_point_x = map.dist_trashhold * np.cos(map.segment_pose[closest_direction_id])
        target_point_y = map.dist_trashhold * np.sin(map.segment_pose[closest_direction_id])
        target_point_x += current_pose[0]
        target_point_y += current_pose[1]
        
        # При этом берем высоту от целевой точки, так как данный алгоритм работает только в плоскости.
        return [[target_point_x, target_point_y, target_pose[2]]]
    
    def _find_closest_direction_id(self, current_pose: list, target_pose: list, map: Histogram2d) -> int:
        # Рассчитаем направление на целевую точку.
        target_yaw = np.arctan2(target_pose[1]-current_pose[1],
                                target_pose[0]-current_pose[0])
        # Пройдемся по всем сегментам гистограммы и найдем ближайший сегмент.
        # к целевому направлению. Начнем с индекса 0.
        closest_direction_id = -1
        best_cost = 0
        for index, is_occupied in enumerate(map.bin_hist):
            if not is_occupied: # Если сегмент свободен сравним угловое расстояние при помощи метода _angular_distance
                # В данном примере предлагается использовать в качестве критерия выбора ближайшее направление на целевую точку.
                # Однако можно добавить стоимость изменения направления ЛА или ширину сегмента в качестве дополнительных критериев.
                current_cost = abs(self._angular_distance(target_yaw, map.segment_pose[index]))
                if closest_direction_id == -1:
                    closest_direction_id = index
                    best_cost = current_cost
                # Проверим что разница в стоимости сегментов не превышает порог.
                # Этот порог позволяет избежать многократных переключений между сегментами гистограммы.
                elif (current_cost - best_cost) < -self._direction_treshold:
                    closest_direction_id = index
                    best_cost = current_cost
        return closest_direction_id

    def _angular_distance(self, angle1: float, angle2: float) -> float:
        # Получим разницу между углами(углы должны быть в радианах)
        difference = angle2 - angle1

        # Получим минимальную дистанцию с учетом положения на окружности
        difference = (difference + np.pi) % (2 * np.pi) - np.pi

        return difference

    
if __name__ == "__main__":
    resolution = 8 # 8 segments
    min_distance = 2.0 # if distance of segment closer then 2 it is occupied
    vfh = VFH2d()