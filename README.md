# Taller-en-Clase-Mapeo-con-SLAM


export IGN_IP=127.0.0.1
cd ~/turtlebot4_ws
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze

Lanza SLAM
ros2 launch turtlebot4_navigation slam.launch.py sync:=false

Lanzar RVIZ
ros2 launch turtlebot4_viz view_robot.launch.py
