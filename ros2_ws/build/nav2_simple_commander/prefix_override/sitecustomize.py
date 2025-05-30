import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nada/Desktop/AutonoFly_VTOL/ros2_ws/install/nav2_simple_commander'
