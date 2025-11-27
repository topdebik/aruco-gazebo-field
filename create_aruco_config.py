import os

def generate_aruco_markers_info():
    # ===== КОНФИГУРАЦИЯ =====
    # Параметры должны совпадать с create_world.py
    NUM_MARKERS = 100  # 10x10 маркеров
    MARKER_SIZE = 0.3  # Размер маркера в метрах
    SPACING = 1.0  # Расстояние между маркерами
    
    # Дополнительные параметры для позиционирования
    OFFSET_X = 0.0  # Смещение по X
    OFFSET_Y = 0.0  # Смещение по Y
    HEIGHT = 0.001  # Высота над землей (z координата)
    
    # Углы Эйлера в радианах (по умолчанию все маркеры лежат горизонтально)
    ANGLE_X = 0.0  # Угол вокруг оси X (крен)
    ANGLE_Y = 0.0  # Угол вокруг оси Y (тангаж) 
    ANGLE_Z = 0.0  # Угол вокруг оси Z (рыскание)
    
    OUTPUT_FILE = os.path.expanduser("~") + "/catkin_ws/src/clover/aruco_pose/map/map.txt"
    
    print(f"Генерация информации о {NUM_MARKERS} ArUco маркерах...")
    print(f"Размер маркера: {MARKER_SIZE} м")
    print(f"Расстояние между маркерами: {SPACING} м")
    
    with open(OUTPUT_FILE, "w") as f:
        markers_per_row = 10
        
        for i in range(NUM_MARKERS):
            row = i // markers_per_row
            col = i % markers_per_row
            
            # Вычисляем координаты (совпадает с create_world.py)
            x = OFFSET_X + col * SPACING
            y = OFFSET_Y + row * SPACING
            z = HEIGHT
            
            # Форматируем строку: id size x y z angle_z angle_y angle_x
            line = f"{i} {MARKER_SIZE:.3f} {x:.3f} {y:.3f} {z:.3f} {ANGLE_Z:.3f} {ANGLE_Y:.3f} {ANGLE_X:.3f}"
            f.write(line + "\n")
    
    print(f"✅ Файл с информацией о маркерах создан: {OUTPUT_FILE}")

if __name__ == "__main__":
    generate_aruco_markers_info()