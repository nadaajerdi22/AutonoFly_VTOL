# 🛸 AutonoFly VTOL Simulation

This project provides a complete simulation of an autonomous VTOL drone using ROS2, PX4, Gazebo, QGroundControl, and navigation algorithms such as RRT and FASTSLAM2. 

-- 
The goal is to simulate a drone that can:
- Localize itself in the environment using FastSLAM2
- Automatically plan a path using RRT
- Navigate autonomously in a simulated world using PX4 and Gazebo
---

## 📦 Project Structure

- `ros2_ws/`: ROS2 workspace containing:
  - `autonofly_slam`: SLAM node using FastSLAM2
  - `autonofly_nav`: Navigation node using RRT
  - `px4_ros_com`: PX4-ROS2 bridge
- `PX4/`: Custom PX4 firmware with a VTOL drone model
- `ROS2-px4-gazebo-docker/`: Docker setup for ROS2 + PX4 + Gazebo environment
  
---

## 🚨 Requirements
- Docker 
- Linux system or wsl on windows
- QGroundControl for mavlink communication 

---


clone the repo 

```
git clone https://github.com/farashasystemsteam/AutonoFly_VTOL.git 
```
--- 


## 🛠️ Build Docker Image

First, We build our docker image via this command

```bash
docker build -t ros2-px4-gazebo -f complete:v0.2.Dockerfile .
```

## Run Docker Container

To Allow xhost access, We type : 
```
xhost +local:root
```
To Run the docker container

```
docker run -it --rm --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/nada/Desktop/AutonoFly_VTOL/ros2_ws:/root/ros2_ws" \
  --privileged \
  --entrypoint /bin/bash \
  ros2-px4-gazebo
``` 
## inside the Docker container 


We source the workspace
``` 
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
./start_autonomy.sh

```
## here we launch the simulation 
```
ros2 launch autonofly_bringup start_autonomy.launch.py
```

