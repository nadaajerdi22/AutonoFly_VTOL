#!/bin/bash

BASE_DIR=~/Desktop/AutonoFly_VTOL

# Charger l'environnement ROS2
source $BASE_DIR/ros2_ws/install/setup.bash

echo "[+] Lancement de PX4 SITL..."
cd $BASE_DIR/PX4-Autopilot && \
make px4_sitl gz_standard_vtol_AutonoFly_VTOL  &
PX4_PID=$!

sleep 10  # Laisse PX4 démarrer correctement

echo "[+] Lancement du noeud px4_ros_com..."
ros2 run px4_ros_com px4_ros_com_node  &
PX4_ROS_COM_PID=$!

sleep 1

echo "[+] Lancement du noeud FastSLAM2..."
ros2 run autonofly_slam fastslam_node &
FASTSLAM_PID=$!

echo "[+] Lancement du noeud RRT..."
ros2 run autonofly_nav rrt_node &
RRT_PID=$!

echo
echo "🚀 Simulation complète lancée."
echo "📌 Pour arrêter : Ctrl+C"
echo

# Attente des processus (supprimer GAZEBO_PID qui n'existe pas ici)
wait $PX4_PID $PX4_ROS_COM_PID $FASTSLAM_PID $RRT_PID