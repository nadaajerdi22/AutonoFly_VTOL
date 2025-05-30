import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nada/Desktop/AutonoFly_VTOL/ros2_ws/src/autonofly_slam/install/autonofly_slam'
