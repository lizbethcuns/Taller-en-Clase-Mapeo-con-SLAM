# Taller-en-Clase-Mapeo-con-SLAM

# Si el robot se vuelve loco o se para (modo seguro), simplemente cierra todo y vuelve a lanzar el mundo con este comando (solución al problema de tus amigos):
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse lidar:=cpu 

export IGN_IP=127.0.0.1
cd ~/turtlebot4_ws
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze

Lanza SLAM
ros2 launch turtlebot4_navigation slam.launch.py sync:=false

Lanzar RVIZ
ros2 launch turtlebot4_viz view_robot.launch.py

MOVER DESDE EL TERMINAL
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.7 , y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# -------------- En otra terminal -------------
source /opt/ros/humble/setup.bash
# Guardar el Mapa
cd turtlebot4_ws
cd src

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"                                              EN ESTA LINEA HAY QUE PONERLE EL NOMBRE QUE YO QUIERA

# ------------------- ENTENDER ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --rate 10 --------------------------------
#x: 0.7 ------------- avanza hacia adelante a 0.7 metros por segundo (m/s)
#y: 0.0 y z: 0.0 evitan movimiento lateral o vertical (el robot es 2D
#(angular): Controla la rotación. Aquí todo es 0, así que no gira
#--rate 10: Envía el comando 10 veces por segundo (10 Hz)




import os
import cv2
import csv
import yaml
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Map, SearchFactory
from pathlib import Path

# ===================== MAP LOADING =====================
def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)

    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()

    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Binarizar
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    # Downsample
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    map_bin = cv2.resize(
        map_bin,
        (w // downsample_factor, h // downsample_factor),
        interpolation=cv2.INTER_AREA
    )

    if downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)

    resolution *= downsample_factor
    return map_bin, resolution, origin

# ===================== MAP → RRT ENV =====================
def map_from_binary(map_bin):
    h, w = map_bin.shape
    env = Map(w, h)
    obs_rect = []

    for y in range(h):
        for x in range(w):
            if map_bin[y, x] == 1:
                obs_rect.append([x, h - 1 - y, 1, 1])

    env.update(obs_rect=obs_rect)
    return env

# ===================== COORDENADAS =====================
def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)

def map_to_world(x_map, y_map, resolution, origin):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)

# ===================== WAYPOINTS 0.5 m =====================
def resample_path_rrt(path, resolution, origin, step=0.5):
    if not path:
        return []

    resampled = [path[0]]
    last_xw, last_yw = map_to_world(*path[0], resolution, origin)

    for x_map, y_map in path[1:]:
        xw, yw = map_to_world(x_map, y_map, resolution, origin)
        dist = np.hypot(xw - last_xw, yw - last_yw)

        if dist >= step:
            resampled.append((x_map, y_map))
            last_xw, last_yw = xw, yw

    if path[-1] not in resampled:
        resampled.append(path[-1])

    return resampled

# ===================== SAVE CSV =====================
def save_path_as_csv(path, filename, resolution, origin):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in path:
            x, y = map_to_world(x_map, y_map, resolution, origin)
            writer.writerow([x, y])

# ===================== MAIN =====================
if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "Oschersleben_map.yaml"

    downsample_factor = 8              # ======valor de 8

    x_start, y_start = -21.0, -4.0
    x_goal, y_goal   = -19.0, -4.7

    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    env = map_from_binary(map_bin)

    start = world_to_map(x_start, y_start, resolution, origin)
    goal  = world_to_map(x_goal, y_goal, resolution, origin)

    print(f"Start (map): {start}, Goal (map): {goal}")
    print(f"Resolución: {resolution:.4f} m/celda")

    # ===================== RRT =====================
    planner = SearchFactory()(
        "rrt",
        start=start,
        goal=goal,
        env=env,
        max_dist=10,
        sample_num=100000
    )

    planner.run()
    cost, path, _ = planner.plan()

    if path:
        path_05 = resample_path_rrt(path, resolution, origin, step=0.5)
        save_path_as_csv(path_05, "rrt_05m.csv", resolution, origin)
        print(f"Ruta RRT 0.5 m guardada ({len(path_05)} waypoints)")
    else:
        print("RRT no encontró camino") 





import os
import cv2
import csv
import yaml
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Map, SearchFactory
from pathlib import Path

# ===================== MAP LOADING =====================
def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)
    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']
    
    # Binarizar
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1
    
    # Downsample
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    map_bin = cv2.resize(
        map_bin,
        (w // downsample_factor, h // downsample_factor),
        interpolation=cv2.INTER_AREA
    )
    if downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)
    
    # === AQUÍ ESTÁ LA CLAVE: DILATACIÓN DESPUÉS DEL DOWNSAMPLE ===
    # Esto cierra los huecos y hace el mapa más robusto
    safety_margin_meters = 0.4  # margen de seguridad
    kernel_size = max(1, int(safety_margin_meters / (resolution * downsample_factor)) * 2 + 1)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    map_bin = cv2.dilate(map_bin, kernel, iterations=1)
    print(f"Dilatación aplicada: kernel {kernel_size}x{kernel_size} ≈ {safety_margin_meters} m")
    
    resolution *= downsample_factor
    return map_bin, resolution, origin

# ===================== MAP → RRT ENV =====================
def map_from_binary(map_bin):
    h, w = map_bin.shape
    env = Map(w, h)
    obs_rect = []
    for y in range(h):
        for x in range(w):
            if map_bin[y, x] == 1:
                obs_rect.append([x, h - 1 - y, 1, 1])
    env.update(obs_rect=obs_rect)
    return env

# ===================== COORDENADAS =====================
def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)

def map_to_world(x_map, y_map, resolution, origin):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)

# ===================== WAYPOINTS =====================
def resample_path_rrt(path, resolution, origin, step=0.5):
    """Genera waypoints cada 'step' metros (0.5 o 1.0)"""
    if not path:
        return []
    resampled = [path[0]]
    last_xw, last_yw = map_to_world(*path[0], resolution, origin)
    for x_map, y_map in path[1:]:
        xw, yw = map_to_world(x_map, y_map, resolution, origin)
        dist = np.hypot(xw - last_xw, yw - last_yw)
        if dist >= step - 0.01:  # tolerancia pequeña
            resampled.append((x_map, y_map))
            last_xw, last_yw = xw, yw
    if path[-1] not in resampled:
        resampled.append(path[-1])
    return resampled

# ===================== SAVE CSV =====================
def save_path_as_csv(path, filename, resolution, origin):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in path:
            x, y = map_to_world(x_map, y_map, resolution, origin)
            writer.writerow([x, y])

# ===================== MAIN =====================
if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "Oschersleben_map.yaml"
    downsample_factor = 8

    x_start, y_start = -21.0, -4.0
    x_goal, y_goal = -19.0, -4.7

    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    env = map_from_binary(map_bin)

    start = world_to_map(x_start, y_start, resolution, origin)
    goal = world_to_map(x_goal, y_goal, resolution, origin)
    print(f"Start (map): {start}, Goal (map): {goal}")
    print(f"Resolución: {resolution:.4f} m/celda")

    planner = SearchFactory()(
        "rrt",
        start=start,
        goal=goal,
        env=env,
        max_dist=10,
        sample_num=100000
    )
    planner.run()
    cost, path, _ = planner.plan()

    if path:
        # Waypoints cada 0.5 metros
        path_05 = resample_path_rrt(path, resolution, origin, step=0.5)
        save_path_as_csv(path_05, "rrt_05m.csv", resolution, origin)
        print(f"RRT 0.5 m: {len(path_05)} waypoints → rrt_05m.csv")

        # Waypoints cada 1.0 metro
        path_1m = resample_path_rrt(path, resolution, origin, step=1.0)
        save_path_as_csv(path_1m, "rrt_1m.csv", resolution, origin)
        print(f"RRT 1.0 m: {len(path_1m)} waypoints → rrt_1m.csv")
    else:
        print("RRT no encontró camino")
