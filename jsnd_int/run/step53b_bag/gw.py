import rosbag2_py,pickle,sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
T={t.name:get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/global_waypoints']))
t,d,ts=r.read_next(); m=deserialize_message(d,T[t])
pickle.dump([(w.s_m,w.x_m,w.y_m,w.d_right,w.d_left,w.psi_rad,w.kappa_radpm) for w in m.wpnts],open(sys.argv[2],'wb')); print(len(m.wpnts))
