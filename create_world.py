import cv2
import os

def create_aruco_world_with_movement():
    # ===== КОНФИГУРАЦИЯ =====
    WORLD_NAME = "aruco_field"
    BASE_MODEL_PATH = "./aruco_models"
    TEXTURES_SOURCE_DIR = "./textures"
    CAR_MODEL_FILE = "cybertruck_new_textured.dae"
    NUM_MARKERS = 100
    MARKER_SIZE = 0.3
    SPACING = 1.0

    # Границы летного поля
    FIELD_MIN_X = 0.0
    FIELD_MAX_X = 9.0 * SPACING
    FIELD_MIN_Y = 0.0
    FIELD_MAX_Y = 9.0 * SPACING

    # ===== СОЗДАНИЕ СТРУКТУРЫ ПАПОК =====
    materials_scripts_dir = os.path.join(BASE_MODEL_PATH, "materials", "scripts")
    materials_textures_dir = os.path.join(BASE_MODEL_PATH, "materials", "textures")

    os.makedirs(materials_scripts_dir, exist_ok=True)
    os.makedirs(materials_textures_dir, exist_ok=True)

    # ===== СОЗДАНИЕ ФАЙЛА МАТЕРИАЛОВ =====
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

    material_file_path = os.path.join(materials_scripts_dir, "aruco.material")
    with open(material_file_path, "w") as f:
        f.write(full_material_script)

    # ===== ГЕНЕРАЦИЯ ТЕКСТУР =====
    textures_dir = os.path.join("textures")

    # Создаем директорию если не существует
    os.makedirs(os.path.dirname(textures_dir) if os.path.dirname(textures_dir) else '.', exist_ok=True)

    # Используем словарь 4x4
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

    for i in range(NUM_MARKERS):
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, i, int(MARKER_SIZE * 1e3))
        marker_img = cv2.rotate(marker_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite(os.path.join(textures_dir, f"aruco_{i:03d}.png"), marker_img)

    # ===== КОПИРОВАНИЕ ТЕКСТУР =====
    for i in range(NUM_MARKERS):
        src_texture = os.path.join(TEXTURES_SOURCE_DIR, f"aruco_{i:03d}.png")
        dst_texture = os.path.join(materials_textures_dir, f"aruco_{i:03d}.png")
        if os.path.exists(src_texture):
            import shutil
            shutil.copy2(src_texture, dst_texture)

    # ===== СОЗДАНИЕ WORLD-ФАЙЛА =====
    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{WORLD_NAME}">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="car">
      <pose>5 5 0.001 0 0 0</pose>
      <link name="link">
        <gravity>0</gravity>
        <visual name="visual" cast_shadows="true">
          <geometry>
            <mesh>
              <uri>file://{CAR_MODEL_FILE}</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
      
      <plugin name="random_movement" filename="libRandomMovementPlugin.so">
        <field_size_x>9</field_size_x>
        <field_size_y>9</field_size_y>
        <object_size_x>2.0</object_size_x>
        <object_size_y>0.8</object_size_y>
      </plugin>
    </model>
"""

    # Добавляем ArUco маркеры
    markers_per_row = 10
    for i in range(NUM_MARKERS):
        row = i // markers_per_row
        col = i % markers_per_row
        x = col * SPACING
        y = row * SPACING
        z = 0.001

        marker_block = f"""
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
              <uri>model://aruco_models/materials/scripts</uri>
              <uri>model://aruco_models/materials/textures</uri>
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

    world_content += """
  </world>
</sdf>"""

    with open(f"{WORLD_NAME}.world", "w") as f:
        f.write(world_content)

    print("✅ Мир создан: aruco_field.world")
    print("🚀 Для запуска: gazebo aruco_field.world")

if __name__ == "__main__":
    create_aruco_world_with_movement()