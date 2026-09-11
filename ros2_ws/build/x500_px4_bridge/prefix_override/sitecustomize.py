import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deepak/PX4-Autopilot/ros2_ws/install/x500_px4_bridge'
