import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deepak/gps-denied-vtol/ros2_ws/install/x500_px4_bridge'
