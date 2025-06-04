import sys
if sys.prefix == '/home/nada/Desktop/AutonoFly_VTOL/venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nada/Desktop/AutonoFly_VTOL/ros2_ws/install/autonofly_slam'
