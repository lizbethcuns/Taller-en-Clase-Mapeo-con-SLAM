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

ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.7 , y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
