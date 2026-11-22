#!/usr/bin/env python3
"""Read-only bench check (no publishing): control rate, MPC compute, mux/E-stop state.
Usage: python3 ~/JSND_INT/tests_ros/bench_check.py [seconds=10]"""
import sys, time, collections, numpy as np, rclpy
from std_msgs.msg import String, Float32
from ackermann_msgs.msg import AckermannDriveStamped
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
rclpy.init(); n = rclpy.create_node('bench_check_ro')
T = collections.defaultdict(list); last = {}; st = collections.Counter(); el = []; spd = []
def stamp(k): T[k].append(time.monotonic())
def hl(m): stamp('controller'); spd.append(m.drive.speed)
n.create_subscription(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd', hl, 10)
n.create_subscription(AckermannDriveStamped, '/vesc/ackermann_cmd', lambda m: stamp('mux'), 10)
n.create_subscription(String, '/controller/mpc_status', lambda m: st.update([m.data]), 10)
n.create_subscription(Float32, '/controller/mpc_elapsed_ms', lambda m: el.append(m.data), 10)
for k in ('/simple_mux/estop_state', '/simple_mux/active_host'):
    n.create_subscription(String, k, lambda m, k=k: last.__setitem__(k, m.data), 10)
t0 = time.monotonic()
while time.monotonic() - t0 < dur: rclpy.spin_once(n, timeout_sec=0.005)
for k in ('controller', 'mux'):
    t = np.array(T[k])
    if len(t) > 2:
        d = np.diff(t) * 1e3
        print(f'{k:10s} {len(t)/(t[-1]-t[0]):6.1f} Hz  period p50 {np.median(d):.1f} p99 {np.percentile(d,99):.1f} max {d.max():.1f} ms')
    else: print(f'{k:10s} NO DATA')
if el: e = np.array(el); print(f'MPC solve  p50 {np.median(e):.2f} p99 {np.percentile(e,99):.2f} max {e.max():.2f} ms (n={len(e)})')
tot = sum(st.values()); print('mpc_status', ', '.join(f'{k} {100*v/tot:.0f}%' for k, v in st.most_common(5)) if tot else 'NO DATA')
print('controller speed max %.3f m/s' % max(spd) if spd else 'controller speed: NO DATA')
for k in ('/simple_mux/estop_state', '/simple_mux/active_host'): print(k, last.get(k, 'NO DATA'))
