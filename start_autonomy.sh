#!/bin/bash

# ==============================================
# CONFIGURATION - MODIFIEZ CES VARIABLES SI BESOIN
# ==============================================
PX4_DIR="$HOME/Desktop/AutonoFly_VTOL/PX4"
ROS2_WS_DIR="$HOME/Desktop/AutonoFly_VTOL/ros2_ws"
MODEL="standard_vtol"        # Votre modèle VTOL
WORLD="AutonoFly_VTOL"       # Votre monde personnalisé
MAVLINK_PORT=14560           # Port MAVLink
GZ_VERSION="harmonic"        # Version de Gazebo

# ==============================================
# FONCTIONS UTILITAIRES
# ==============================================
start_px4() {
    echo "Lancement de PX4 avec Gazebo..."
    cd "$PX4_DIR" || exit
    source $PX4_DIR/Tools/setup_gazebo.bash
    export PX4_SIM_MODEL=$MODEL
    export GZ_VERSION=$GZ_VERSION
    make px4_sitl "gz_${MODEL}_${WORLD}"
}

start_mavros() {
    echo "Lancement de MAVROS..."
    sleep 10  # Attente du démarrage de PX4
    source /opt/ros/jazzy/setup.bash
    ros2 launch mavros mavros_node.launch.py \
        fcu_url:="udp://:${MAVLINK_PORT}@127.0.0.1:${MAVLINK_PORT}"
}

start_algorithms() {
    echo "Lancement des algorithmes autonomes..."
    sleep 15  # Attente de MAVROS
    export GAZEBO_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/build_gazebo
    export GAZEBO_MODEL_PATH=$PX4_DIR/Tools/simulation/gz/models:$GAZEBO_MODEL_PATH
    ros2 launch autono_fly autonomy.launch.py
}

# ==============================================
# LANCEMENT PRINCIPAL
# ==============================================
start_px4 &          # Lance en arrière-plan
start_mavros &       # Lance en arrière-plan
start_algorithms     # Lance au premier plan

wait  # Attend la fin des processus
echo "Simulation terminée"