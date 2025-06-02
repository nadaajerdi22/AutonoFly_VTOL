#!/bin/bash

set -e  # Arrêter le script si une commande échoue

echo "=== Mise à jour du système ==="
sudo apt update && sudo apt upgrade -y

echo "=== Installation des dépendances système ==="
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    ros-jazzy-desktop \
    gz-harmonic

echo "=== Initialisation rosdep ==="
sudo rosdep init || true
rosdep update

echo "=== Installation de ROS2 packages spécifiques PX4 ==="
# Exemple d'installation d'un package ROS2 PX4
# sudo apt install -y ros-jazzy-px4-msgs

echo "=== Configuration de l'environnement ROS2 ==="
source /opt/ros/jazzy/setup.bash

echo "=== Clone et compilation workspace PX4 ROS2 ==="
# Remplace par ton repo ou workspace
if [ ! -d "ros2_ws" ]; then
    mkdir -p ros2_ws/src
fi

cd ros2_ws/src
# Clone tes dépôts ici si besoin, par exemple :
# git clone https://github.com/PX4/px4_ros_com.git

cd ../
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo "=== Sourcing workspace ==="
source install/setup.bash

echo "=== Installation terminée ==="
