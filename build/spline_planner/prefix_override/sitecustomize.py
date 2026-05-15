import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hanjh/F1_TENTH_UNITA/install/spline_planner'
