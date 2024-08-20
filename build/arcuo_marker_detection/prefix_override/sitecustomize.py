import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sarthak/precise_docking_ws/install/arcuo_marker_detection'
