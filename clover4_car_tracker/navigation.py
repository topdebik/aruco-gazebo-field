"""
Навигация и управление движением дрона
"""
import numpy as np
import math
import os
from config import NAVIGATION_CONFIG, DRONE_CONFIG, MARKERS_FILE_PATH

class NavigationController:
    def __init__(self):
        self.takeoff_position = None
        self.current_position = None
        self.current_yaw = 0
        self.position_tolerance = NAVIGATION_CONFIG['position_tolerance']
        self.yaw_tolerance = NAVIGATION_CONFIG['yaw_tolerance']
        
        # Загрузка карты меток из файла
        self.markers_map = self.load_markers_from_file()
        
        # Калибровочные параметры камеры
        self.camera_params = {
            'focal_length': 500,  # Фокусное расстояние в пикселях
        }
        
    def load_markers_from_file(self):
        """Загрузка координат меток из файла"""
        markers_map = {}
        
        if not os.path.exists(MARKERS_FILE_PATH):
            print(f"Файл меток не найден: {MARKERS_FILE_PATH}")
            return markers_map
        
        try:
            with open(MARKERS_FILE_PATH, 'r') as file:
                for line in file:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # Формат: id_маркера размер_маркера x y z угол_z угол_y угол_x
                    parts = line.split()
                    if len(parts) >= 7:
                        marker_id = int(parts[0])
                        marker_size = float(parts[1])  # Размер маркера в метрах
                        x = float(parts[2])
                        y = float(parts[3])
                        z = float(parts[4])
                        
                        markers_map[marker_id] = {
                            'position': (x, y, z),
                            'size': marker_size
                        }
                        print(f"Метка {marker_id}: ({x:.2f}, {y:.2f}, {z:.2f}), размер: {marker_size}m")
                    
        except Exception as e:
            print(f"Ошибка чтения файла меток: {e}")
        
        return markers_map
        
    def set_takeoff_position(self, position):
        """Установка точки взлета как домашней позиции"""
        self.takeoff_position = position
    
    def update_position(self, position, yaw):
        """Обновление текущей позиции дрона"""
        self.current_position = position
        self.current_yaw = yaw
    
    def find_closest_marker(self, markers_info):
        """Нахождение ближайшего маркера"""
        if not markers_info:
            return None
        
        closest_marker = None
        max_size = 0
        
        for marker in markers_info:
            marker_id = marker['id']
            if marker_id in self.markers_map:
                if marker['size_px'] > max_size:
                    max_size = marker['size_px']
                    closest_marker = marker
        
        return closest_marker
    
    def estimate_car_position_simple(self, markers_info, car_position, frame_size):
        """Упрощенная оценка позиции автомобиля относительно маркеров"""
        if not markers_info or not car_position:
            return None
        
        closest_marker = self.find_closest_marker(markers_info)
        if not closest_marker:
            return None
        
        marker_id = closest_marker['id']
        marker_data = self.markers_map[marker_id]
        marker_world_pos = marker_data['position']
        marker_real_size = marker_data['size']
        
        # Оцениваем расстояние до маркера
        distance_to_marker = (marker_real_size * self.camera_params['focal_length']) / closest_marker['size_px']
        
        # Вычисляем смещение автомобиля относительно маркера
        car_x, car_y = car_position['pixel_x'], car_position['pixel_y']
        marker_x, marker_y = closest_marker['center_x'], closest_marker['center_y']
        
        dx_pixels = car_x - marker_x
        dy_pixels = car_y - marker_y
        
        # Переводим пиксели в метры
        pixels_to_meters = distance_to_marker / self.camera_params['focal_length']
        
        dx_meters = dx_pixels * pixels_to_meters
        dy_meters = dy_pixels * pixels_to_meters
        
        # Предполагаем, что автомобиль на земле (z = 0)
        car_world_x = marker_world_pos[0] + dx_meters
        car_world_y = marker_world_pos[1] + dy_meters
        car_world_z = 0.0
        
        return (car_world_x, car_world_y, car_world_z), closest_marker
    
    def calculate_movement_to_car(self, car_position, car_prediction=None):
        """Вычисление движения к автомобилю"""
        if not car_position:
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        # Базовое движение на основе текущей позиции автомобиля
        # В системе координат дрона:
        # vx - вперед/назад (положительное - вперед)
        # vy - влево/вправо (положительное - влево)
        # vz - вверх/вниз (положительное - вверх)
        
        norm_x = car_position['normalized_x']  # -1 (слева) до 1 (справа)
        norm_y = car_position['normalized_y']  # -1 (вверху) до 1 (внизу)
        
        # Пропорциональный контроллер для движения
        kp_xy = 0.5  # Увеличенный коэффициент для более агрессивного управления
        
        # Корректируем знаки для правильного направления
        vx = norm_y * kp_xy  # Движение вперед/назад (положительное - вперед)
        vy = -norm_x * kp_xy  # Движение влево/вправо (положительное - влево)
        vz = 0  # Поддержание высоты
        
        # Корректировка на основе предсказания движения
        if car_prediction:
            # Упреждающее управление
            prediction_gain = 0.3
            vx += car_prediction['velocity_y'] * prediction_gain
            vy += car_prediction['velocity_x'] * prediction_gain
        
        # Ограничение скорости
        max_speed = DRONE_CONFIG['max_speed']
        vx = np.clip(vx, -max_speed, max_speed)
        vy = np.clip(vy, -max_speed, max_speed)
        vz = np.clip(vz, -max_speed, max_speed)
        
        return {
            'vx': vx,
            'vy': vy, 
            'vz': vz
        }
    
    def calculate_return_home(self):
        """Вычисление движения к точке взлета"""
        if not self.takeoff_position or not self.current_position:
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        # Вычисление вектора к домашней позиции
        dx = self.takeoff_position[0] - self.current_position[0]
        dy = self.takeoff_position[1] - self.current_position[1]
        dz = self.takeoff_position[2] - self.current_position[2]
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < self.position_tolerance:
            # Достигли точки взлета
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        # Пропорциональное управление
        kp = 0.5
        vx = dx * kp
        vy = dy * kp
        vz = dz * kp
        
        # Ограничение скорости
        max_speed = DRONE_CONFIG['max_speed']
        vx = np.clip(vx, -max_speed, max_speed)
        vy = np.clip(vy, -max_speed, max_speed)
        vz = np.clip(vz, -max_speed, max_speed)
        
        return {
            'vx': vx,
            'vy': vy,
            'vz': vz
        }
    
    def is_at_home_position(self):
        """Проверка достижения домашней позиции"""
        if not self.takeoff_position or not self.current_position:
            return False
        
        dx = self.takeoff_position[0] - self.current_position[0]
        dy = self.takeoff_position[1] - self.current_position[1]
        dz = self.takeoff_position[2] - self.current_position[2]
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance < self.position_tolerance
    
    def calculate_distance_to_car(self, car_world_position):
        """Вычисление расстояния до автомобиля"""
        if not car_world_position or not self.current_position:
            return float('inf')
        
        dx = car_world_position[0] - self.current_position[0]
        dy = car_world_position[1] - self.current_position[1]
        dz = car_world_position[2] - self.current_position[2]
        
        return math.sqrt(dx**2 + dy**2 + dz**2)