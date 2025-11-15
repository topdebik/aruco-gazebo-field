import os

TEXTURE_PATH = "/home/clover/aruco_field/worlds/aruco_images"

if not os.path.exists(os.path.join(TEXTURE_PATH, "aruco_000.png")):
    print(f"Not found file aruco_000.png in {TEXTURE_PATH}")
    print("Path is correct and PNG-file create.")
    exit(1)

header = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
'''

footer = '''
  </world>
</sdf>
'''

def marker_block(id, x, y):
    z = 0.0
    size = 0.3
    png = f"aruco_{id:03d}.png"
    return f'''
    <model name="aruco_marker_{id}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{size} {size} 0.001</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/White</name>
            </script>
            <shader type="pixel"/>
            <texture>
              <filename>file://{TEXTURE_PATH}/{png}</filename>
            </texture>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>{size} {size} 0.001</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
'''
with open("aruco_field.world", "w") as f:
    f.write(header)
    id = 0
    for y in range(10):        # str y=0 (bottom row) → y=9 (upper)
        for x in range(10):    # columns: x=0 (left) → x=9 (right)
            f.write(marker_block(id, x * 1.0, y * 1.0))
            id += 1
    f.write(footer)

print("World 'aruco_field.world' create!")
print("Launch: gazebo aruco_field.world")