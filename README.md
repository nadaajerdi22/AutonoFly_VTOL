# 🛸 AutonoFly VTOL Simulation

## 📦 Build Docker Image

```bash
docker build -t ros2-px4-gazebo -f complete:v0.2.Dockerfile .
```

## Run Docker Container
```
xhost +local:root
```
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
## inside the contqiner 
``` 
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
```
## here we launch the simulation 
```
ros2 launch autonofly_bringup start_autonomy.launch.py
```
