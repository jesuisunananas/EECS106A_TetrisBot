import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cc/ee106a/fa25/class/ee106a-ahn/ros_workspaces/proj/EECS106A_TetrisBot/lab7/install/planning'
