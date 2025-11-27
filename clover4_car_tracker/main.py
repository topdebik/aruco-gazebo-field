"""
Главный файл для управления дроном Clover 4 при отслеживания автомобиля
"""
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger

from drone_controller import DroneController
from camera_processor import CameraProcessor
from car_tracker import CarTracker
from navigation import NavigationController
from config import DRONE_CONFIG, DEBUG_MODE

class CarTrackingMission:
    def __init__(self):
        # Инициализация ROS ноды
        if not rospy.get_node_uri():
            rospy.init_node('car_tracker')
        
        # Инициализация компонентов
        self.drone = DroneController()
        self.camera = CameraProcessor()
        self.tracker = CarTracker()
        self.navigation = NavigationController()
        self.bridge = CvBridge()
        
        # Сервисы
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        
        # Состояние миссии
        self.mission_state = "INIT"
        self.start_time = None
        self.following_start_time = None
        self.current_frame = None
        
        # Подписка на камеру
        self.image_sub = rospy.Subscriber('/clover/front_camera/image_raw', Image, self.image_callback)
        print("Подписка на камеру создана: /clover/front_camera/image_raw")
        
        # Инициализация окон отладки
        if DEBUG_MODE:
            cv2.namedWindow('Car Mask', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Detection Results', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Car Mask', 640, 480)
            cv2.resizeWindow('Detection Results', 640, 480)
            print("Отладочные окна инициализированы")
        
    def image_callback(self, msg):
        """Обработка изображения с камеры"""
        try:
            # Конвертируем ROS Image в OpenCV изображение
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            print(f"Ошибка обработки изображения: {e}")
    
    def wait_for_camera(self, timeout=10):
        """Ожидание получения первого кадра с камеры"""
        print("Ожидание кадра с камеры...")
        start_time = time.time()
        while self.current_frame is None and time.time() - start_time < timeout:
            print("Кадр не получен, ожидание...")
            rospy.sleep(0.5)
        
        if self.current_frame is not None:
            print("Кадр с камеры получен успешно")
            return True
        else:
            print("Таймаут ожидания кадра с камеры")
            return False
    
    def run_mission(self):
        """Основной цикл выполнения миссии"""
        print("Запуск миссии отслеживания автомобиля...")
        
        try:
            # Ожидание кадра с камеры перед началом миссии
            if not self.wait_for_camera():
                print("Не удалось получить кадр с камеры. Завершение миссии.")
                return
            
            # Этап 1: Взлет
            self.mission_state = "TAKEOFF"
            print("Этап 1: Взлет...")
            takeoff_pos = self.drone.takeoff()
            if not takeoff_pos:
                print("Ошибка взлета. Завершение миссии.")
                return
            
            self.navigation.set_takeoff_position(takeoff_pos)
            print("Взлет завершен успешно")
            
            # Небольшая пауза для стабилизации
            rospy.sleep(2)
            
            # Этап 2: Поиск автомобиля
            self.mission_state = "SEARCHING"
            print("Этап 2: Поиск автомобиля...")
            car_found = self.search_for_car(timeout=30)
            
            if not car_found:
                print("Автомобиль не найден. Возврат и посадка.")
                self.return_and_land()
                return
            
            # Этап 3: Приближение к автомобилю
            self.mission_state = "APPROACHING"
            print("Этап 3: Приближение к автомобилю...")
            self.approach_car()
            
            # Этап 4: Следование за автомобилем
            self.mission_state = "FOLLOWING"
            print("Этап 4: Следование за автомобилем...")
            self.follow_car()
            
            # Этап 5: Возврат и посадка
            self.mission_state = "RETURNING"
            print("Этап 5: Возврат к точке взлета...")
            self.return_and_land()
            
            print("Миссия завершена успешно!")
            
        except KeyboardInterrupt:
            print("Миссия прервана пользователем")
            self.emergency_land()
        except Exception as e:
            print(f"Ошибка в миссии: {e}")
            self.emergency_land()
        finally:
            # Закрытие окон при завершении
            if DEBUG_MODE:
                cv2.destroyAllWindows()
    
    def search_for_car(self, timeout=60):
        """Поиск автомобиля с вращением дрона"""
        start_time = time.time()
        search_direction = 1  # Направление вращения
        
        print("Начало поиска автомобиля с вращением...")
        
        while time.time() - start_time < timeout:
            if self.current_frame is None:
                print("Нет кадра с камеры, ожидание...")
                rospy.sleep(0.5)
                continue
            
            # Обработка кадра
            frame = self.camera.preprocess_frame(self.current_frame)
            car_contours, car_mask = self.camera.detect_pink_car(frame)
            car_position = self.camera.calculate_car_position(car_contours)
            
            # Детекция маркеров для отладки
            markers_info, _ = self.camera.detect_aruco_markers(frame)
            closest_marker = self.navigation.find_closest_marker(markers_info) if markers_info else None
            
            # Отладочная визуализация
            if DEBUG_MODE:
                self.camera.create_debug_visualization(frame, car_contours, car_mask, markers_info, car_position, closest_marker)
            
            if car_position:
                print("Автомобиль найден!")
                # Остановка вращения
                self.drone.set_yaw_rate(0)
                # Небольшая пауза для стабилизации
                rospy.sleep(1)
                return True
            
            # Вращение для поиска
            print(f"Вращение дрона, направление: {search_direction}")
            success = self.drone.set_yaw_rate(0.5 * search_direction)  # Увеличил скорость вращения
            
            if not success:
                print("Ошибка установки скорости вращения")
            
            rospy.sleep(0.2)  # Увеличил задержку
            
            # Смена направления каждые 10 секунд
            if time.time() - start_time > 10:
                search_direction *= -1
                print(f"Смена направления вращения на: {search_direction}")
                start_time = time.time()
        
        # Остановка вращения при выходе по таймауту
        self.drone.set_yaw_rate(0)
        print("Поиск автомобиля завершен по таймауту")
        return False
    
    def approach_car(self):
        """Приближение к автомобилю"""
        approach_timeout = 60
        start_time = time.time()
        target_distance = DRONE_CONFIG['following_distance']
        
        print("Начало приближения к автомобилю...")
        
        while time.time() - start_time < approach_timeout:
            if self.current_frame is None:
                print("Нет кадра с камеры, ожидание...")
                rospy.sleep(0.5)
                continue
            
            # Обновление телеметрии
            telem = self.drone.get_current_telemetry()
            if telem:
                self.navigation.update_position((telem['x'], telem['y'], telem['z']), telem['yaw'])
            
            # Обработка кадра
            frame = self.camera.preprocess_frame(self.current_frame)
            
            # Детекция маркеров
            markers_info, _ = self.camera.detect_aruco_markers(frame)
            
            # Детекция автомобиля
            car_contours, car_mask = self.camera.detect_pink_car(frame)
            car_position = self.camera.calculate_car_position(car_contours)
            
            if car_position:
                self.tracker.update_position(car_position)
                
                # Если есть маркеры, оцениваем позицию автомобиля
                car_world_position = None
                closest_marker = None
                
                if markers_info:
                    result = self.navigation.estimate_car_position_simple(
                        markers_info, car_position, (frame.shape[1], frame.shape[0])
                    )
                    if result:
                        car_world_position, closest_marker = result
                
                # Отладочная визуализация
                if DEBUG_MODE:
                    self.camera.create_debug_visualization(frame, car_contours, car_mask, markers_info, car_position, closest_marker)
                
                # Вычисление управления
                control = self.navigation.calculate_movement_to_car(car_position)
                print(f"Управление: vx={control['vx']:.2f}, vy={control['vy']:.2f}, vz={control['vz']:.2f}")
                
                # Применение управления
                success = self.drone.set_velocity_command(
                    vx=control['vx'],
                    vy=control['vy'],
                    vz=control['vz']
                )
                
                if not success:
                    print("Ошибка установки скорости, пробуем удержание позиции")
                    self.drone.hold_position(0.1)
                
                # Проверка достижения целевой дистанции
                if car_world_position:
                    current_distance = self.check_approximation_distance(car_world_position)
                    print(f"Дистанция до автомобиля: {current_distance:.2f} м")
                    
                    if current_distance <= target_distance and current_distance > 0:
                        print(f"Достигнута целевая дистанция: {current_distance:.2f} м")
                        self.drone.set_velocity_command(0, 0, 0)
                        rospy.sleep(1)
                        break
                else:
                    # Если нет маркеров, используем эмпирический подход
                    if car_position['size_px'] > 200:  # Эмпирический порог
                        print("Достигнута целевая дистанция (эмпирически)")
                        self.drone.set_velocity_command(0, 0, 0)
                        rospy.sleep(1)
                        break
            else:
                # Автомобиль не обнаружен - остановка
                self.drone.set_velocity_command(0, 0, 0)
                
                # Отладочная визуализация даже если автомобиль не найден
                if DEBUG_MODE:
                    markers_info, _ = self.camera.detect_aruco_markers(frame)
                    self.camera.create_debug_visualization(frame, car_contours, car_mask, markers_info, None, None)
                
                print("Автомобиль не обнаружен при приближении")
            
            rospy.sleep(0.1)
    
    def follow_car(self):
        """Следование за автомобилем в течение 30 секунд"""
        self.following_start_time = time.time()
        following_duration = DRONE_CONFIG['following_time']
        
        print("Начало следования за автомобилем...")
        
        while time.time() - self.following_start_time < following_duration:
            if self.current_frame is None:
                print("Нет кадра с камеры, ожидание...")
                rospy.sleep(0.5)
                continue
            
            # Обновление телеметрии
            telem = self.drone.get_current_telemetry()
            if telem:
                self.navigation.update_position((telem['x'], telem['y'], telem['z']), telem['yaw'])
            
            # Обработка кадра
            frame = self.camera.preprocess_frame(self.current_frame)
            
            # Детекция маркеров и автомобиля
            markers_info, _ = self.camera.detect_aruco_markers(frame)
            car_contours, car_mask = self.camera.detect_pink_car(frame)
            car_position = self.camera.calculate_car_position(car_contours)
            
            if car_position:
                self.tracker.update_position(car_position)
                
                # Находим ближайший маркер для отладки
                closest_marker = self.navigation.find_closest_marker(markers_info) if markers_info else None
                
                # Отладочная визуализация
                if DEBUG_MODE:
                    self.camera.create_debug_visualization(frame, car_contours, car_mask, markers_info, car_position, closest_marker)
                
                # Вычисление управления для следования
                control = self.navigation.calculate_movement_to_car(car_position)
                print(f"Следование: vx={control['vx']:.2f}, vy={control['vy']:.2f}")
                
                # Применение управления
                success = self.drone.set_velocity_command(
                    vx=control['vx'],
                    vy=control['vy'], 
                    vz=control['vz']
                )
                
                if not success:
                    print("Ошибка установки скорости при следовании")
                    self.drone.hold_position(0.1)
            else:
                # Автомобиль потерян - удержание позиции
                self.drone.set_velocity_command(0, 0, 0)
                
                # Отладочная визуализация даже если автомобиль не найден
                if DEBUG_MODE:
                    self.camera.create_debug_visualization(frame, car_contours, car_mask, markers_info, None, None)
                
                print("Автомобиль не обнаружен при следовании. Ожидание...")
            
            # Отображение оставшегося времени
            remaining = following_duration - (time.time() - self.following_start_time)
            if remaining % 5 < 0.1:  # Выводим каждые ~5 секунд
                print(f"Следование: {remaining:.1f} сек осталось")
            
            rospy.sleep(0.1)
        
        # Остановка после завершения следования
        self.drone.set_velocity_command(0, 0, 0)
        print("Следование завершено")
    
    def check_approximation_distance(self, car_world_position):
        """Проверка достижения целевой дистанции до автомобиля"""
        if not car_world_position:
            return float('inf')
        
        return self.navigation.calculate_distance_to_car(car_world_position)
    
    def return_and_land(self):
        """Возврат к точке взлета и посадка"""
        print("Возврат к точке взлета...")
        
        return_timeout = 60
        start_time = time.time()
        
        while time.time() - start_time < return_timeout:
            # Обновление телеметрии
            telem = self.drone.get_current_telemetry()
            if telem:
                self.navigation.update_position((telem['x'], telem['y'], telem['z']), telem['yaw'])
            
            # Вычисление пути домой
            control = self.navigation.calculate_return_home()
            
            # Применение управления
            success = self.drone.set_velocity_command(
                vx=control['vx'],
                vy=control['vy'],
                vz=control['vz']
            )
            
            if not success:
                print("Ошибка установки скорости при возврате")
                self.drone.hold_position(0.1)
            
            # Проверка достижения домашней позиции
            if self.navigation.is_at_home_position():
                print("Достигнута точка взлета")
                self.drone.set_velocity_command(0, 0, 0)
                rospy.sleep(1)
                break
            
            rospy.sleep(0.1)
        
        # Посадка
        self.mission_state = "LANDING"
        print("Посадка...")
        self.drone.land_drone()
    
    def emergency_land(self):
        """Аварийная посадка"""
        print("Аварийная посадка!")
        self.drone.set_velocity_command(0, 0, 0)
        self.drone.land_drone()

if __name__ == "__main__":
    mission = CarTrackingMission()
    mission.run_mission()