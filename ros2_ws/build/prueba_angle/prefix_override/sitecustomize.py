import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/crofi/AutoModelCrofi/ros2_ws/install/prueba_angle'
