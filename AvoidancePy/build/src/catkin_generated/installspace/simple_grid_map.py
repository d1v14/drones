import os
import struct
import numpy as np

from functools import partial

class GridMap3D:
    def __init__(self, x_size: int=10, y_size: int=10, z_size: int=10,
                 origin_x: int=50, origin_y: int=50,
                 origin_z: int=3, resolution: float=1):
        # Устанавливаем новые параметры карты
        self.reset(x_size, y_size, z_size, origin_x, origin_y, origin_z, resolution)
    
    def reset(self, x_size: int, y_size: int, z_size: int,
            origin_x: int, origin_y: int, origin_z: int, resolution: float):
        # Инициализируем карту 
        # Задаем размерность
        self._x_size = x_size
        self._y_size = y_size
        self._z_size = z_size
        # Устанавливаем разрешение карты(размеры вокселей) в метрах
        self._resolution = resolution
        # Так как в нашей карте воксели имеют кубический растр то все их стороны одинаковые
        self._cell_size_x = resolution
        self._cell_size_y = resolution
        self._cell_size_z = resolution
        # Устанавливаем индекс точки относительно которой будет производиться рассчет 
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._origin_z = origin_z
        # Определяем положение точки отсчета, далее эта информация будеи использована для смещения
        # положения в карте. В противном случае координаты будут задаваться от первого элкмента карты
        self._origin_x_pose = float(self._origin_x * resolution)
        self._origin_y_pose = float(self._origin_y * resolution)
        self._origin_z_pose = float(self._origin_z * resolution)

        # Создаем массив заданной размерности N x M x K. Этот ма
        self.grid = [[[0 for _ in range(z_size)] for _ in range(y_size)] for _ in range(x_size)]

    def get_cell_coordinates(self, x: float, y: float, z: float) -> tuple:
        # Переводим координаты с плавающей точкой, в порядковый номер ячейки x y z
        cell_x = int((x + self._origin_x_pose) / float(self._cell_size_x))
        cell_y = int((y + self._origin_y_pose) / float(self._cell_size_y))
        cell_z = int((z + self._origin_z_pose) / float(self._cell_size_z))
        return cell_x, cell_y, cell_z
    
    def get_cell_pose_by_ids(self, ids: tuple) -> tuple:
        x = (ids[0] * self._cell_size_x) - self._origin_x_pose
        y = (ids[1] * self._cell_size_y) - self._origin_y_pose
        z = (ids[2] * self._cell_size_z) - self._origin_z_pose
        return (x, y ,z)

    def set_value(self, x: float, y: float, z: float, value: float) -> None:
        # Устанавливаем значение ячейки по заданным координатам
        cell_x, cell_y, cell_z = self.get_cell_coordinates(x, y, z)
        # Проверяем что порядковые номера ячеек не выходят за границы массива, в противном случае
        # игнорируем установку значение. Для усовершенствования можно добавить в таком случае методы
        # для расширения массива или его сдвига(например при использовании циклического буфера)
        if 0 <= cell_x < self._x_size and 0 <= cell_y < self._y_size and 0 <= cell_z < self._z_size:
            self.grid[cell_x][cell_y][cell_z] = value

    def get_value(self, x: float, y: float, z: float) -> float:
        # Получаем значение ячейки по координатам
        cell_x, cell_y, cell_z = self.get_cell_coordinates(x, y, z)
        # Проверяем что порядковые номера ячеек не выходят за границы массива
        if 0 <= cell_x < self._x_size and 0 <= cell_y < self._y_size and 0 <= cell_z < self._z_size:
            return self.grid[cell_x][cell_y][cell_z]
        else:
            return None
        
    def get_value_by_indexes(self, cell_x: int, cell_y: int, cell_z: int) -> float:
        # Получаем значение ячейки по заданному индексу
        if self.is_index_valid([cell_x, cell_y, cell_z]):
            return self.grid[cell_x][cell_y][cell_z]
        else:
            return None
        
    def save_to_file(self, filename: str) -> None:
        with open(filename, "wb") as f:
            f.write(struct.pack('iiiiiif', self._x_size, self._y_size, self._z_size,
                                self._origin_x, self._origin_y, self._origin_z,
                                self._resolution))
            for x, y, z, value in self:
                f.write(struct.pack('ffff', x, y, z, value))
    
    def read_from_file(self, filename: str) -> None:
        with open(filename, "rb") as f:
            data = f.read(28)  # Read the entire binary data
            x_size, y_size, z_size, \
            origin_x, origin_y, origin_z, \
            resolution = struct.unpack('iiiiiif', data)
            
            self.reset(x_size, y_size, z_size, origin_x, origin_y, origin_z, resolution)
            
            while True:
                data = f.read(16)  # Read 28 bytes
                if not data or len(data) != 16:
                    break  # End of file reached
                
                x, y, z, value = struct.unpack('ffff', data)
                self.set_value(x, y, z, value)
    
    def is_occupied(self, cell_coords: tuple):
        return np.isclose(1.0, map.get_value_by_indexes(cell_coords[0],
                                                        cell_coords[1],
                                                        cell_coords[2]))
    
    def is_index_valid(self, cell_coords: tuple) -> bool:
        # Этот метод проверяет не выходит ли заданный индекс за пределы массива
        return 0 <= cell_coords[0] < self._x_size and 0 <= cell_coords[1] < \
                self._y_size and 0 <= cell_coords[2] < self._z_size
    
    def check_collisions_between_points(self, first_point: tuple, second_point: tuple) -> bool:
        # Получаем все узлы в диапазоне от точки до точки
        # При наличии препятствия хотя бы в одной из ячеек возвращаем True
        return False
    
    def check_collisions_in_area(self, point: tuple, radius: float) -> bool:
        return False
        
    
    @property
    def size(self) -> tuple:
        return (self._x_size, self._y_size, self._z_size)
    
    def __len__(self) -> int:
        return self._x_size * self._y_size * self._z_size
            
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
            if self.x_iter < self._x_size:
                # Получаем координаты блока в зависимости от точки отсчета(origin)
                # и систем координат.
                x = (self.x_iter * self._cell_size_x) - self._origin_x_pose
                y = (self.y_iter * self._cell_size_y) - self._origin_y_pose
                z = (self.z_iter * self._cell_size_z) - self._origin_z_pose
                value = self.grid[self.x_iter][self.y_iter][self.z_iter]
                
                # Выполняем сдвиг порядкового номера элемента на каждом шаге
                self.z_iter += 1
                if self.z_iter >= self._z_size:
                    self.z_iter = 0
                    self.y_iter += 1
                    if self.y_iter >= self._y_size:
                        self.y_iter = 0
                        self.x_iter += 1
                # Возвращаем координаты блока и значение в нем.
                return x, y, z, value 
            else:
                raise StopIteration
                

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map = GridMap3D(20, 20, 10, 5, 5, 0, 1)

    # set first level
    for x, y, z, value in map:
        if z <= 0:
            map.set_value(x, y, z, 1.0) 
        elif x == 1 and y == 2:
            map.set_value(x, y, z, 1.0)
        elif x == 3 and y == 2:
            map.set_value(x, y, z, 1.0)
        elif x == -4 and y == 5:
            map.set_value(x, y, z, 1.0)
        
        elif x == 0 and y == 5:
            map.set_value(x, y, z, 1.0)
            map.set_value(x+1, y, z, 1.0)
            
        elif x == -2 and y == 7:
            map.set_value(x, y, z, 1.0)
            map.set_value(x+1, y, z, 1.0)   
            
        elif x == 4 and y == 8:
            map.set_value(x, y, z, 1.0)
            map.set_value(x+1, y, z, 1.0) 
    
    map.save_to_file(script_dir + "/map_example.map")
    
    map.read_from_file(script_dir + "/map_example.map")