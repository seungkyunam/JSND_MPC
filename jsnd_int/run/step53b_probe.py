import json, time, collections, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool, Float32
from f110_msgs.msg import BehaviorStrategy, ObstacleArray
from nav_msgs.msg import Odometry
D=collections.defaultdict(list)
class N(Node):
    def __init__(s):
        super().__init__('s53c_probe')
        s.create_subscription(BehaviorStrategy,'/behavior_strategy',lambda m:D['bs'].append(m),10)
        s.create_subscription(String,'/controller/mpc_diagnostics',lambda m:D['diag'].append(json.loads(m.data)),10)
        s.create_subscription(String,'/controller/mpc_status',lambda m:D['st'].append(m.data),10)
        s.create_subscription(Float32,'/controller/mpc_elapsed_ms',lambda m:D['ms'].append(m.data),10)
        s.create_subscription(ObstacleArray,'/tracking/obstacles',lambda m:D['obs'].append(m),10)
        s.create_subscription(Odometry,'/car_state/odom_frenet',lambda m:D['fr'].append(m),10)
        s.create_subscription(String,'/simple_mux/active_host',lambda m:D['host'].append(m.data),10)
        s.create_subscription(Bool,'/simple_mux/estop_state',lambda m:D['estop'].append(m.data),10)
rclpy.init(); n=N(); t=time.time()
while time.time()-t<3: rclpy.spin_once(n,timeout_sec=0.05)
bs=D['bs']; print('bs n=%d hz=%.1f'%(len(bs),len(bs)/5))
print('states',collections.Counter(m.state for m in bs))
if bs:
    m=bs[-1]; d=np.array([w.d_m for w in m.local_wpnts]); s=np.array([w.s_m for w in m.local_wpnts])
    print('last: nwp=%d s[%.2f..%.2f] d min %.3f max %.3f'%(len(d),s.min(),s.max(),d.min(),d.max()))
    print(' d profile',[(round(w.s_m,1),round(w.d_m,3)) for w in m.local_wpnts[::6]]); print(' dl/dr at max|d|:',[(round(w.s_m,2),round(w.d_m,3),round(w.d_left,3),round(w.d_right,3)) for w in [m.local_wpnts[int(np.argmax(abs(d)))]]])
    print(' overtaking',[(round(o.s_center,2),round(o.d_center,3),round(o.size,2),o.is_static) for o in m.overtaking_targets],'trailing',len(m.trailing_targets))
    dmax=[float(np.max(np.abs([w.d_m for w in x.local_wpnts]))) if x.local_wpnts else 0 for x in bs]; print(' max|d| per msg min/max %.3f %.3f'%(min(dmax),max(dmax)))
if D['obs']:
    print('tracking obs',[(round(o.s_center,2),round(o.d_center,3),round(o.d_left,3),round(o.d_right,3),round(o.size,2),o.is_static) for o in D['obs'][-1].obstacles])
if D['fr']:
    f=D['fr'][-1].pose.pose.position; print('car frenet s=%.2f d=%.3f'%(f.x,f.y))
print('mpc status',collections.Counter(D['st']).most_common(6))
if D['ms']: print('mpc ms p50 %.2f p99 %.2f n=%d'%(np.percentile(D['ms'],50),np.percentile(D['ms'],99),len(D['ms'])))
if D['diag']: print('diag last',{k:v for k,v in D['diag'][-1].items() if k!='predicted_speed_mps'})
print('host',collections.Counter(D['host']),'estop',collections.Counter(D['estop']))
m=bs[-1]
print('avoid seg (s,d,dL,dR,vx):',[(round(w.s_m,2),round(w.d_m,3),round(w.d_left,3),round(w.d_right,3),round(w.vx_mps,2)) for w in m.local_wpnts if 7.0<=w.s_m<=9.0][::3])
