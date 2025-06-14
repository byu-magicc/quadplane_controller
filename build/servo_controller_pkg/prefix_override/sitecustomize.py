import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/quadplane/quadplane_ctrl_ws/install/servo_controller_pkg'
