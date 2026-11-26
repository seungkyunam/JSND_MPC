import sys, json, pickle, rosbag2_py, math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={t.name:t.type for t in r.get_all_topics_and_types()}
skip={'/scan','/behavior_strategy','/global_waypoints','/state_machine/trailing_target'}
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in types if t not in skip]))
cls={t:get_message(ty) for t,ty in types.items()}
D={}; bad=0
def add(t,row): D.setdefault(t,[]).append(row)
while r.has_next():
    t,d,ts=r.read_next(); ts*=1e-9
    try: m=deserialize_message(d,cls[t])
    except Exception: bad+=1; continue
    if t in('/controller/mpc_diagnostics','/state_machine/debug'):
        j=json.loads(m.data)
        if t=='/state_machine/debug':
            j={k:j.get(k) for k in('state','cur_s','cur_d','cur_vs','n_obs','close_to_raceline')}|{'gb':j.get('gb_free'),'pd':j.get('planner_diag'),'sa_reinit':(j.get('static_avoidance') or {}).get('reinit_count')}
        add(t,(ts,j))
    elif t in('/controller/mpc_status','/state_machine','/simple_mux/estop_state','/simple_mux/active_host'): add(t,(ts,m.data))
    elif 'ackermann' in t: add(t,(ts,m.drive.speed,m.drive.steering_angle,m.drive.acceleration))
    elif t in('/car_state/odom','/pf/pose/odom'):
        q=m.pose.pose.orientation; y=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        add(t,(ts,m.pose.pose.position.x,m.pose.pose.position.y,y,m.twist.twist.linear.x,m.twist.twist.angular.z))
    elif t=='/car_state/odom_frenet': add(t,(ts,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.linear.y))
    elif t=='/vesc/sensors/core': s=m.state; add(t,(ts,s.speed,s.current_motor,s.duty_cycle))
    elif t=='/vesc/sensors/imu_corrected': add(t,(ts,m.linear_acceleration.x,m.linear_acceleration.y,m.angular_velocity.z))
    elif t.startswith('/controller/mpc_') : add(t,(ts,m.data))
    elif t=='/tracking/obstacles': add(t,(ts,[(o.id,o.s_center,o.d_center,o.is_static,o.vs) for o in m.obstacles]))
    elif t=='/local_waypoints':
        w=m.wpnts; add(t,(ts,w[0].s_m if w else None,[(p.vx_mps,p.kappa_radpm) for p in w[:30:3]]))
print('bad',bad,{k:len(v) for k,v in D.items()})
pickle.dump(D,open('d.pkl','wb'))
