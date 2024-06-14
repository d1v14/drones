import numpy as np

class Histogram2d:
    def __init__(self) -> None:
        self._range_hist = None
        self._bin_hist = None
        self._hist_start = None
        self._hist_end = None
        self._is_init = False
        self._dist_trash = 0.0
        self._seg_pose = None
        
    @property
    def segment_pose(self):
        return (self._seg_pose)
        
    @property
    def dist_trashhold(self):
        return self._dist_trash
    
    @property
    def range_hist(self):
        return self._range_hist
    
    @property
    def bin_hist(self):
        return self._bin_hist
    
    @property
    def histogram_step(self):
        return
     
    @property
    def segment_resolution(self):
        return self._hist_segment_res
        
    def init_histogram(self, resolution: int = 8,
                 max_range: float=10, min_range: float=0.1,
                 dist_trash: float = 2,
                 start_angle: float = -np.pi,
                 end_angle: float = np.pi):
        # Установим разрешение гистограммы
        self._resolution = resolution
        # Установим минимальную и максимальную дальности
        self._min_range = min_range
        self._max_range = max_range
        # Установим пороговое значение при котором препятствие будет считаться обнаруженным
        self._dist_trash = dist_trash
        # Установим начальное и конечное угловое положение для измерений
        self._hist_start = start_angle
        self._hist_end = end_angle
        
        # Рассчитаем угловое разрешение сегмента гистограммы
        self._hist_segment_res = (self._hist_end - self._hist_start) / self._resolution

        # Сдвинем гистограмму на половину углового разрешения сегмента чтобы он смотрел вперед по направлению аппарата
        self._shift = (self._hist_segment_res / self._resolution) / 2.0
        
        # Создадим массивы под гистограмму дальностей и бинарную гистограмму
        self._range_hist = np.zeros(self._resolution, dtype=np.float32)
        self._bin_hist = np.zeros(self._resolution, dtype=np.bool_())
        
        # Создадим массив для хранения углового положения сегментов гистограммы
        self._seg_pose = np.linspace(self._hist_start, self._hist_end,
                                self._resolution, endpoint=False)
        
        self._is_init = True
        
    @property
    def is_init(self) -> bool:
        return self._is_init
        
    def update_histogram(self, ranges) -> None:
        # Обновим гистограмму для планирования маршрута
        # Получим количество измерений в одном сегменте гистограммы
        range_step = (self._hist_end - self._hist_start) / len(ranges)
        # Создадим локальные переменные для гистограммы дальностей и бинарной гистограммы
        bin_hist = np.zeros(self._resolution, dtype=np.bool_())
        range_hist = np.zeros(self._resolution, dtype=np.float32)
        
        # Пройдемся по всему массиву дальностей и заполним гистограмму дальностей
        # Получим минимальные измерения для каждого сегмента для каждого столбца гистограммы
        # При этом отбросим нулевые значения как не валидные
        for id, range in enumerate(ranges):
            hist_id = int((id * range_step + self._shift) /  self._hist_segment_res) % self._resolution
            if range < range_hist[hist_id] or range_hist[hist_id] == 0.0:
                range_hist[hist_id] = range
        range_hist[range_hist < self._min_range] = 0
        range_hist[range_hist > self._max_range] = 0
        
        # Выполним бинаризацию гистограммы. Если величина столбца больше порога self._dist_trash или
        # равна 0 будем считать сегмент свободным(False), в противном случае занятым(True)
        bin_hist[(range_hist < self._dist_trash) & (~np.isclose(range_hist, 0.0, atol=1e-9))] = True

        # Обновим гистограммы
        self._range_hist = range_hist                 
        self._bin_hist = bin_hist
    
    def __len__(self):
        return len(self._range_hist)
        
        
     
     
 