import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/josep/PycharmProjects/ModelDownload/ros2_ws/install/ia_to_ros'
