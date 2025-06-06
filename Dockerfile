# Use ROS 2 Humble Desktop as the base image
FROM osrf/ros:humble-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    sudo \
    wget \
    curl \
    tmux \
    ruby \
    tmuxinator



# Setup Micro XRCE-DDS Agent & Client
RUN cd /root && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/

# Install Python requirements. If you don't have gpu, uncomment next line -torch cpu installation-
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install \
    mavsdk \
    aioconsole \
    pygame \
    opencv-python \
    ultralytics

RUN apt-get install -y ros-humble-ros-gzgarden

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 uninstall -y numpy

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/


# Modify camera angle
RUN sed -i 's|<pose>.12 .03 .242 0 0 0</pose>|<pose>.15 .029 .21 0 0.7854 0</pose>|' /root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf

# Additional Configs
RUN echo "source /root/ws_sensor_combined/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc

# Copy tmuxinator configuration
COPY all_in_one.yml /root/.config/tmuxinator/px4_ros2_gazebo.yml

# Set up tmuxinator
RUN echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc

# Set default command to start tmuxinator
CMD ["tmuxinator", "start", "px4_ros2_gazebo"]