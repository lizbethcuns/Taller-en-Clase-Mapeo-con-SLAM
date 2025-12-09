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

