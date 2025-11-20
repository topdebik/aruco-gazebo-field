import os


def create_aruco_world_with_materials():
    # ===== КОНФИГУРАЦИЯ =====
    WORLD_NAME = "aruco_field"
    BASE_MODEL_PATH = "./aruco_models"  # Базовая папка для модели
    TEXTURES_SOURCE_DIR = (
        "/home/debik/aruco-gazebo-field/textures"  # Папка с PNG-файлами
    )
    NUM_MARKERS = 100  # 10x10 маркеров
    MARKER_SIZE = 0.3  # Размер маркера в метрах
    SPACING = 1.0  # Расстояние между маркерами

    # ===== СОЗДАНИЕ СТРУКТУРЫ ПАПОК =====
    materials_scripts_dir = os.path.join(BASE_MODEL_PATH, "materials", "scripts")
    materials_textures_dir = os.path.join(BASE_MODEL_PATH, "materials", "textures")

    os.makedirs(materials_scripts_dir, exist_ok=True)
    os.makedirs(materials_textures_dir, exist_ok=True)

    # ===== СОЗДАНИЕ ФАЙЛА МАТЕРИАЛОВ =====
    # Создаем полный скрипт материалов для всех маркеров
    full_material_script = "// ArUco markers material definitions\n\n"
    for i in range(NUM_MARKERS):
        full_material_script += f"""material Aruco/Material_{i:03d}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture aruco_{i:03d}.png
        filtering anisotropic
        max_anisotropy 16
      }}
    }}
  }}
}}

"""

    # Сохраняем файл материалов
    material_file_path = os.path.join(materials_scripts_dir, "aruco.material")
    with open(material_file_path, "w") as f:
        f.write(full_material_script)

    # ===== ПРОВЕРКА НАЛИЧИЯ ТЕКСТУР =====
    missing_textures = []
    for i in range(NUM_MARKERS):
        src_texture = os.path.join(TEXTURES_SOURCE_DIR, f"aruco_{i:03d}.png")
        dst_texture = os.path.join(materials_textures_dir, f"aruco_{i:03d}.png")

        if not os.path.exists(src_texture):
            missing_textures.append(f"aruco_{i:03d}.png")
        else:
            # Копируем текстуру в целевую директорию
            import shutil

            shutil.copy2(src_texture, dst_texture)

    if missing_textures:
        print("Обнаружены отсутствующие текстуры:")
        for texture in missing_textures:
            print(f"   - {texture}")
        print(
            "Скрипт продолжит работу, но соответствующие маркеры не будут отображаться правильно"
        )

    # ===== СОЗДАНИЕ WORLD-ФАЙЛА =====
    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{WORLD_NAME}">
    <!-- Освещение -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Земля -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
"""

    # Добавляем ArUco маркеры
    markers_per_row = 10

    for i in range(NUM_MARKERS):
        row = i // markers_per_row
        col = i % markers_per_row

        x = col * SPACING
        y = row * SPACING
        z = 0.001  # Немного выше земли

        marker_block = f"""
    <!-- ArUco маркер {i:03d} -->
    <model name="aruco_marker_{i:03d}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{MARKER_SIZE} {MARKER_SIZE} 0.001</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://{os.path.abspath(materials_scripts_dir)}</uri>
              <uri>file://{os.path.abspath(materials_textures_dir)}</uri>
              <name>Aruco/Material_{i:03d}</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>{MARKER_SIZE} {MARKER_SIZE} 0.001</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
"""
        world_content += marker_block

    # Завершаем world файл
    world_content += """
  </world>
</sdf>"""

    # Сохраняем world файл
    world_file_path = f"{WORLD_NAME}.world"
    with open(world_file_path, "w") as f:
        f.write(world_content)

if __name__ == "__main__":
    create_aruco_world_with_materials()
    print("To run: gazebo aruco_field.world")
