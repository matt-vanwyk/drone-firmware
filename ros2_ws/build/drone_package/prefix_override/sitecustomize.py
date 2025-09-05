import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/spiderweb/autonomous-drone-system/drone-firmware/ros2_ws/install/drone_package'
