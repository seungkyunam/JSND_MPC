import sys, rosbag2_py, pickle
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/global_waypoints']))
t,d,ts=r.read_next(); m=deserialize_message(d,get_message('f110_msgs/msg/WpntArray'))
W=[(w.s_m,w.x_m,w.y_m,w.psi_rad,w.kappa_radpm,w.vx_mps,w.ax_mps2,w.d_left,w.d_right) for w in m.wpnts]
pickle.dump(W,open('gw.pkl','wb')); print(len(W),W[-1][0])
