"""
Трекинг автомобиля и предсказание траектории
"""
import numpy as np
import cv2
from collections import deque
import time

class CarTracker:
    def __init__(self, buffer_size=50):
        self.position_buffer = deque(maxlen=buffer_size)
        self.time_buffer = deque(maxlen=buffer_size)
        self.last_update_time = None
        self.is_tracking = False
        
    def update_position(self, car_position):
        """Обновление позиции автомобиля"""
        current_time = time.time()
        
        if car_position:
            self.position_buffer.append(car_position)
            self.time_buffer.append(current_time)
            self.last_update_time = current_time
            self.is_tracking = True
        else:
            self.is_tracking = False
    
    def predict_movement(self, prediction_time=1.0):
        """Предсказание движения автомобиля"""
        if len(self.position_buffer) < 3:
            return None
        
        # Получаем последние позиции
        positions = list(self.position_buffer)
        times = list(self.time_buffer)
        
        # Вычисляем скорость и направление
        recent_positions = positions[-5:]  # Последние 5 позиций
        recent_times = times[-5:]
        
        if len(recent_positions) < 2:
            return None
        
        # Вычисляем среднюю скорость
        velocities = []
        for i in range(1, len(recent_positions)):
            dt = recent_times[i] - recent_times[i-1]
            if dt > 0:
                dx = recent_positions[i]['normalized_x'] - recent_positions[i-1]['normalized_x']
                dy = recent_positions[i]['normalized_y'] - recent_positions[i-1]['normalized_y']
                velocities.append((dx/dt, dy/dt))
        
        if not velocities:
            return None
        
        avg_velocity = np.mean(velocities, axis=0)
        
        # Предсказываем будущую позицию
        last_position = positions[-1]
        predicted_x = last_position['normalized_x'] + avg_velocity[0] * prediction_time
        predicted_y = last_position['normalized_y'] + avg_velocity[1] * prediction_time
        
        # Ограничиваем значения
        predicted_x = np.clip(predicted_x, -1, 1)
        predicted_y = np.clip(predicted_y, -1, 1)
        
        return {
            'normalized_x': predicted_x,
            'normalized_y': predicted_y,
            'velocity_x': avg_velocity[0],
            'velocity_y': avg_velocity[1]
        }
    
    def get_tracking_quality(self):
        """Оценка качества трекинга"""
        if len(self.position_buffer) < 10:
            return 0.0
        
        # Анализируем стабильность позиций
        positions = list(self.position_buffer)[-10:]
        x_positions = [p['normalized_x'] for p in positions]
        y_positions = [p['normalized_y'] for p in positions]
        
        # Вычисляем дисперсию
        x_variance = np.var(x_positions)
        y_variance = np.var(y_positions)
        
        # Качество обратно пропорционально дисперсии
        quality = 1.0 / (1.0 + x_variance + y_variance)
        return min(quality, 1.0)
    
    def reset(self):
        """Сброс трекера"""
        self.position_buffer.clear()
        self.time_buffer.clear()
        self.is_tracking = False