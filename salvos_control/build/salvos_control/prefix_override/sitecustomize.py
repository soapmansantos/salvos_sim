import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/santos/dev_ws/src/salvos_control/install/salvos_control'
