#!/bin/bash
make -C ~/Desktop/AutonoFly_VTOL/PX4 px4_sitl gz_standard_vtol_AutonoFly_VTOL &
sleep 10
source ~/Desktop/AutonoFly_VTOL/ros2_ws/install/setup.bash
ros2 launch autono_fly autonomy.launch.py