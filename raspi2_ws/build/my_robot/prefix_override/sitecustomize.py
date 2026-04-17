import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/viethung/Downloads/robot_follow_lane/raspi2_ws/install/my_robot'
