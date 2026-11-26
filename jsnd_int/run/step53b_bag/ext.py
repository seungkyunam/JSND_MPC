import sys, json, pickle, math, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p,out=sys.argv[1],sys.argv[2]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=p,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
T={t.name:get_message(t.type) for t in r.get_all_topics_and_types()}
D={k:[] for k in 'host est sm diag elap cyc per fr od pf cmd hcmd core obs bs scan dbg'.split()}
bsn=0
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
while r.has_next():
    t,d,ts=r.read_next(); tb=ts*1e-9
    if t not in T: continue
    m=deserialize_message(d,T[t])
    if t=='/simple_mux/active_host': D['host'].append((tb,m.data))
    elif t=='/simple_mux/estop_state': D['est'].append((tb,m.data))
    elif t=='/state_machine': D['sm'].append((tb,m.data))
    elif t=='/controller/mpc_diagnostics':
        j=json.loads(m.data); j['tb']=j.pop('predicted_speed_mps',None) and tb or tb; j.pop('optimal_accel_horizon_mps2',None); D['diag'].append(j)
    elif t=='/controller/mpc_elapsed_ms': D['elap'].append((tb,m.data))
    elif t=='/controller/mpc_cycle_ms': D['cyc'].append((tb,m.data))
    elif t=='/controller/mpc_loop_period_ms': D['per'].append((tb,m.data))
    elif t=='/car_state/odom_frenet': D['fr'].append((tb,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x))
    elif t=='/car_state/odom':
        P=m.pose.pose; st=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        D['od'].append((tb,st,P.position.x,P.position.y,yaw(P.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z))
    elif t=='/pf/pose/odom':
        P=m.pose.pose; st=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        D['pf'].append((tb,st,P.position.x,P.position.y,yaw(P.orientation)))
    elif t=='/vesc/ackermann_cmd': D['cmd'].append((tb,m.drive.speed,m.drive.steering_angle))
    elif t=='/vesc/high_level/ackermann_cmd': D['hcmd'].append((tb,m.drive.speed,m.drive.steering_angle))
    elif t=='/vesc/sensors/core': D['core'].append((tb,m.state.speed,m.state.current_motor,m.state.voltage_input))
    elif t=='/tracking/obstacles': D['obs'].append((tb,[(o.s_start,o.s_end,o.d_right,o.d_left,o.x_m,o.y_m,o.is_static,o.id) for o in m.obstacles]))
    elif t=='/behavior_strategy':
        bsn+=1
        if bsn%5==0: D['bs'].append((tb,[(w.s_m,w.d_m,w.x_m,w.y_m) for w in m.local_wpnts]))
    elif t=='/state_machine/debug':
        bsn2=len(D['dbg'])
        j=json.loads(m.data); D['dbg'].append((tb,j.get('state'),j.get('cur_s'),j.get('cur_d'),json.dumps(j.get('static_free')),json.dumps(j.get('planner_diag'))))
    elif t=='/scan':
        rg=m.ranges; D['scan'].append((tb,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.angle_min,m.angle_increment,list(rg)))
pickle.dump(D,open(out,'wb'))
print({k:len(v) for k,v in D.items()})
