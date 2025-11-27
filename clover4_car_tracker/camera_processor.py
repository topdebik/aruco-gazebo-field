"""
Обработка изображений с камеры дрона
"""
import cv2
import numpy as np
from config import PINK_COLOR_RANGE, GREEN_COLOR_RANGE, CAMERA_CONFIG, DEBUG_MODE

class CameraProcessor:
    def __init__(self):
        self.frame_width = CAMERA_CONFIG['frame_width']
        self.frame_height = CAMERA_CONFIG['frame_height']
        self.min_contour_area = CAMERA_CONFIG['min_contour_area']
        
        # Инициализация детектора ArUco маркеров для новой версии OpenCV
        try:
            # Для OpenCV 4.7.0 и новее
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict)
            self.use_new_aruco_api = True
        except AttributeError:
            # Для старых версий OpenCV
            try:
                self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.use_new_aruco_api = False
            except Exception as e:
                print(f"Ошибка инициализации детектора ArUco: {e}")
                self.aruco_dict = None
        
    def preprocess_frame(self, frame):
        """Предобработка кадра"""
        # Изменение размера
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        # Размытие для уменьшения шума
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        return blurred
    
    def detect_aruco_markers(self, frame):
        """Детекция ArUco маркеров на кадре"""
        if self.aruco_dict is None:
            return [], frame
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        try:
            if self.use_new_aruco_api:
                # Новая версия OpenCV
                corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            else:
                # Старая версия OpenCV
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )
        except Exception as e:
            print(f"Ошибка детекции маркеров: {e}")
            return [], frame
        
        markers_info = []
        if ids is not None:
            for i, marker_id in enumerate(ids):
                # Вычисление центра маркера
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                
                # Вычисление размера маркера в пикселях (приблизительно)
                width = np.linalg.norm(corner[0] - corner[1])
                height = np.linalg.norm(corner[1] - corner[2])
                size_px = max(width, height)
                
                markers_info.append({
                    'id': int(marker_id[0]),
                    'center_x': center_x,
                    'center_y': center_y,
                    'corners': corners[i],
                    'size_px': size_px,
                    'pixel_area': cv2.contourArea(corners[i][0])
                })
        
        return markers_info, frame
    
    def detect_pink_car(self, frame):
        """Обнаружение розового автомобиля"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Создание маски для розового цвета
        lower_pink = np.array(PINK_COLOR_RANGE['lower'])
        upper_pink = np.array(PINK_COLOR_RANGE['upper'])
        mask = cv2.inRange(hsv, lower_pink, upper_pink)
        
        # Морфологические операции для улучшения маски
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Поиск контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        car_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_contour_area:
                car_contours.append(contour)
        
        return car_contours, mask
    
    def create_debug_visualization(self, frame, car_contours, mask, markers_info, car_position, closest_marker=None):
        """Создание отладочной визуализации"""
        if not DEBUG_MODE:
            return
        
        # Окно 1: Маска автомобиля
        mask_visual = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Окно 2: Детекции на основном кадре
        result_frame = frame.copy()
        
        # Отрисовка контура автомобиля
        if car_contours:
            largest_contour = max(car_contours, key=cv2.contourArea)
            cv2.drawContours(result_frame, [largest_contour], -1, (255, 0, 0), 2)
            
            if car_position:
                cv2.circle(result_frame, (car_position['pixel_x'], car_position['pixel_y']), 
                          5, (255, 0, 0), -1)
                cv2.putText(result_frame, "Car", (car_position['pixel_x'] + 10, 
                            car_position['pixel_y']), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Отрисовка маркеров
        if markers_info:
            for marker in markers_info:
                try:
                    # Отрисовка контура маркера
                    if self.use_new_aruco_api:
                        cv2.aruco.drawDetectedMarkers(result_frame, [marker['corners']])
                    else:
                        cv2.aruco.drawDetectedMarkers(result_frame, [marker['corners']], 
                                                    np.array([[marker['id']]]))
                    
                    # Отрисовка ID маркера
                    cv2.putText(result_frame, f"ID:{marker['id']}", 
                               (marker['center_x'] + 10, marker['center_y']), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # Выделение ближайшего маркера
                    if closest_marker and marker['id'] == closest_marker['id']:
                        cv2.circle(result_frame, (marker['center_x'], marker['center_y']), 
                                  10, (0, 255, 255), 3)
                        cv2.putText(result_frame, "CLOSEST", 
                                   (marker['center_x'] + 10, marker['center_y'] + 20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        
                        # Линия от маркера к автомобилю
                        if car_position:
                            cv2.line(result_frame, 
                                    (marker['center_x'], marker['center_y']),
                                    (car_position['pixel_x'], car_position['pixel_y']),
                                    (0, 255, 255), 2)
                
                except Exception as e:
                    print(f"Ошибка отрисовки маркера: {e}")
        
        # Добавление информационного текста
        if car_position:
            cv2.putText(result_frame, f"Car size: {car_position['size_px']:.1f}px", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if markers_info:
            cv2.putText(result_frame, f"Markers found: {len(markers_info)}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Показ окон
        cv2.imshow('Car Mask', mask_visual)
        cv2.imshow('Detection Results', result_frame)
        cv2.waitKey(1)
    
    def detect_green_license_plate(self, frame):
        """Обнаружение зеленого номерного знака"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Создание маски для зеленого цвета
        lower_green = np.array(GREEN_COLOR_RANGE['lower'])
        upper_green = np.array(GREEN_COLOR_RANGE['upper'])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Морфологические операции
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        license_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Меньшая площадь для номерного знака
                # Проверка формы (номерной знак обычно прямоугольный)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                if 2.0 < aspect_ratio < 5.0:  # Типичное соотношение для номерных знаков
                    license_contours.append(contour)
        
        return license_contours, mask
    
    def calculate_car_position(self, contours):
        """Вычисление позиции автомобиля в кадре"""
        if not contours:
            return None
        
        # Берем самый большой контур
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Вычисляем центр масс
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Нормализованные координаты (от -1 до 1)
            norm_x = (cx / self.frame_width - 0.5) * 2
            norm_y = (cy / self.frame_height - 0.5) * 2
            
            # Вычисление размера автомобиля в пикселях
            x, y, w, h = cv2.boundingRect(largest_contour)
            car_size_px = max(w, h)
            
            return {
                'pixel_x': cx,
                'pixel_y': cy,
                'normalized_x': norm_x,
                'normalized_y': norm_y,
                'contour': largest_contour,
                'width_px': w,
                'height_px': h,
                'size_px': car_size_px,
                'area_px': cv2.contourArea(largest_contour)
            }
        
        return None