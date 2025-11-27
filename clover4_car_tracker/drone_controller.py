"""
Управление дроном Clover 4 через ROS
"""
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
from config import DRONE_CONFIG

class DroneController:
    def __init__(self):
        # Инициализация ROS сервисов для Clover
        if not rospy.get_node_uri():
            rospy.init_node('car_tracker')
        
        # Сервисы для управления дроном
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
        
        self.takeoff_altitude = DRONE_CONFIG['takeoff_altitude']
        self.landing_speed = DRONE_CONFIG['landing_speed']
        
    def navigate_wait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=False, tolerance=0.1):
        """Блокирующая версия navigate"""
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        
        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
        rospy.sleep(0.1)
    
    def takeoff(self):
        """Взлет дрона с блокировкой"""
        try:
            print(f"Взлет на высоту {self.takeoff_altitude} м...")
            
            # Используем блокирующую версию для взлета
            self.navigate_wait(x=0, y=0, z=self.takeoff_altitude, 
                             speed=0.5, frame_id='body', auto_arm=True, tolerance=0.1)
            
            # Получение позиции взлета
            telem = self.get_telemetry(frame_id='aruco_map')
            takeoff_pos = (telem.x, telem.y, telem.z)
            
            print(f"Взлет завершен. Позиция: {takeoff_pos}")
            return takeoff_pos
            
        except Exception as e:
            print(f"Ошибка при взлете: {e}")
            return None
    
    def land_drone(self):
        """Посадка дрона"""
        try:
            print("Начало посадки...")
            self.land()
            
            # Ждем завершения посадки
            rospy.sleep(5)
            print("Посадка завершена")
            return True
        except Exception as e:
            print(f"Ошибка при посадке: {e}")
            return False
    
    def set_velocity_command(self, vx, vy, vz):
        """Установка скорости движения через set_velocity"""
        try:
            # Используем set_velocity для установки скорости
            self.set_velocity(vx=vx, vy=vy, vz=vz, frame_id='body')
            return True
        except Exception as e:
            print(f"Ошибка установки скорости: {e}")
            return False
    
    def set_yaw_rate(self, yaw_rate):
        """Установка скорости поворота"""
        try:
            self.set_rates(yaw_rate=yaw_rate)
            return True
        except Exception as e:
            print(f"Ошибка установки скорости поворота: {e}")
            return False
    
    def navigate_to_point(self, x, y, z, yaw=0, speed=0.5):
        """Навигация к конкретной точке с блокировкой"""
        try:
            self.navigate_wait(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id='aruco_map')
            return True
        except Exception as e:
            print(f"Ошибка навигации к точке: {e}")
            return False
    
    def get_current_telemetry(self):
        """Получение текущей телеметрии"""
        try:
            telem = self.get_telemetry(frame_id='aruco_map')
            return {
                'x': telem.x,
                'y': telem.y, 
                'z': telem.z,
                'yaw': telem.yaw,
                'vx': telem.vx,
                'vy': telem.vy,
                'vz': telem.vz
            }
        except Exception as e:
            print(f"Ошибка получения телеметрии: {e}")
            return None
    
    def hold_position(self, duration=1.0):
        """Удержание текущей позиции"""
        try:
            telem = self.get_telemetry(frame_id='aruco_map')
            self.navigate(x=telem.x, y=telem.y, z=telem.z, yaw=float('nan'), 
                         frame_id='aruco_map')
            rospy.sleep(duration)
            return True
        except Exception as e:
            print(f"Ошибка удержания позиции: {e}")
            return False