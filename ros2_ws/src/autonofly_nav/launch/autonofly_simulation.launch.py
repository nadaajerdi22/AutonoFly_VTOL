from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Chemins vers les dossiers de tes packages installés ou sources
    gazebo_ros_pkg = os.path.join(os.getenv('HOME'), 'Desktop/AutonoFly_VTOL/ros2_ws/install/gazebo_ros/share/gazebo_ros')
    px4_ros_pkg = os.path.join(os.getenv('HOME'), 'Desktop/AutonoFly_VTOL/ros2_ws/install/px4_ros_com/share/px4_ros_com')
    
    # Chemin vers ton monde Gazebo (.world)
    world_file = os.path.join(os.getenv('HOME'), 'Desktop/AutonoFly_VTOL/PX4/Tools/simulation/gz', 'AutonoFly_VTOL.sdf')

    # Chemin vers l’exécutable PX4 SITL (adapter si nécessaire)
    px4_executable = os.path.join(os.getenv('HOME'), 'Desktop/AutonoFly_VTOL/PX4', 'build/px4_sitl_default/bin/px4')

    return LaunchDescription([
        # Lancer Gazebo avec le monde choisi
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file}.items()
        ),

        # Lancer PX4 SITL (simulateur de vol)
        ExecuteProcess(
            cmd=['px4', px4_executable],
            output='screen'
        ),

        # Lancer ton nœud SLAM (modifie le package et executable selon ton code)
        Node(
            package='autonofly_slam',
            executable='slam_node',
            output='screen'
        ),

        # Lancer ton nœud de navigation
        Node(
            package='autonofly_nav',
            executable='nav_node',
            output='screen'
        ),

        # Lancer RViz2 pour visualiser les données (optionnel)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(os.getenv('HOME'), 'Desktop/AutonoFly_VTOL/ros2_ws/src/autonofly_nav/rviz', 'autonofly_config.rviz')],
            output='screen'
        )
    ])
