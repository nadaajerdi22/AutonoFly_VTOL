# Étape 1 : Base Ubuntu 22.04
FROM ubuntu:22.04

# Étape 2 : Installer les outils de base nécessaires (dont add-apt-repository) en mode non interactif
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    lsb-release sudo curl wget gnupg2 locales tzdata software-properties-common \
    build-essential git vim bash-completion python3-pip \
    ninja-build exiftool cmake gfortran \
    python3-jinja2 python3-toml python3-numpy python3-yaml python3-dev \
    python3-matplotlib python3-scipy python3-opencv python3-pyproj && \
    locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8


# Étape 3 : Ajouter le dépôt ROS 2 Jazzy proprement
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /etc/apt/keyrings/ros.asc > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/ros.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update

# Étape 4 : Installer ROS 2 Jazzy desktop et paquets Python ROS
RUN apt-get install -y ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall \
    python3-pyserial python3-empy

# Étape 5 : Configurer ROS2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO=jazzy

# Étape 6 : Installer PX4
WORKDIR /home/user
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
WORKDIR /home/user/PX4-Autopilot
RUN make px4_sitl_default

# Étape 7 : Copier ros2_ws (algorithmes de navigation)
COPY ./ros2_ws /home/user/ros2_ws
WORKDIR /home/user/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Étape 8 : Copier et configurer QGroundControl
COPY ./QGroundControl.AppImage /home/user/QGroundControl.AppImage
RUN chmod +x /home/user/QGroundControl.AppImage

# Étape 9 : Configurer le répertoire de travail et la commande par défaut
WORKDIR /home/user
CMD ["/bin/bash"]
