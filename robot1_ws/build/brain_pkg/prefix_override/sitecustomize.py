import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/viethung/Downloads/Robot/robot1_ws/install/brain_pkg'
