import rclpy, time, numpy as np, collections
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, String
from ackermann_msgs.msg import AckermannDriveStamped
rclpy.init(); n = rclpy.create_node('bench_check_int'); d = collections.defaultdict(list)
for t in ['/controller/mpc_loop_period_ms', '/controller/mpc_cycle_ms', '/controller/mpc_elapsed_ms']:
    n.create_subscription(Float32, t, lambda m, t=t: d[t].append(m.data), 50)
n.create_subscription(String, '/controller/mpc_status', lambda m: d['status'].append(m.data), 50)
n.create_subscription(String, '/simple_mux/estop_state', lambda m: d['estop'].append(m.data), 10)
n.create_subscription(AckermannDriveStamped, '/vesc/ackermann_cmd', lambda m: d['vesc_speed'].append(m.drive.speed), 50)
s = time.time()
while time.time()-s < 30: rclpy.spin_once(n, timeout_sec=.1)
for k, v in d.items():
    if k in ('status', 'estop'): print(k, collections.Counter(v).most_common(4))
    elif v: a = np.array(v); print(k, len(a), 'mean %.2f p99 %.2f max %.2f' % (a.mean(), np.percentile(a, 99), a.max()))
