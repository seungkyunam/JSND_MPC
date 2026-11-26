#!/usr/bin/env python3
"""Speed-ramp report (step 19) from a speed_rec.sh bag: tracking, braking, steering, MPC.
usage: python3 speed_report.py <bag_dir> [--vmax 3.0]
Only autodrive segments with actual speed > 0.5 m/s count for tracking/steering.
Braking event = mux output speed cmd drops to <=0.05 while actual speed > 1.0 m/s;
distance/time measured until actual speed < 0.1 m/s (car_state/odom pose).
Criteria are suggestions for the go/no-go to the next profile; nothing is changed."""
import sys, math, argparse, bisect, json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('--vmax', type=float, default=None)
a = ap.parse_args()
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
odom, fren, out, ctl, host, estop, st, el = [], [], [], [], [], [], [], []
mpc_diagnostics = []
while r.has_next():
    topic, raw, t = r.read_next(); t *= 1e-9; m = deserialize_message(raw, types[topic])
    if topic == '/car_state/odom':
        p = m.pose.pose.position; odom.append((t, p.x, p.y, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif topic == '/car_state/odom_frenet': fren.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
    elif topic == '/vesc/ackermann_cmd': out.append((t, m.drive.speed, m.drive.steering_angle))
    elif topic == '/vesc/high_level/ackermann_cmd': ctl.append((t, m.drive.speed, m.drive.steering_angle))
    elif topic == '/simple_mux/active_host': host.append((t, m.data))
    elif topic == '/simple_mux/estop_state': estop.append((t, m.data))
    elif topic == '/controller/mpc_status': st.append((t, m.data))
    elif topic == '/controller/mpc_elapsed_ms': el.append((t, m.data))
    elif topic == '/controller/mpc_diagnostics':
        try:
            mpc_diagnostics.append((t, json.loads(m.data)))
        except (ValueError, TypeError):
            pass

def at(seq, t, default=None):
    i = bisect.bisect_right([s[0] for s in seq], t) - 1 if seq else -1
    return seq[i] if i >= 0 else default
ht = [h[0] for h in host]
def is_auto(t):
    i = bisect.bisect_right(ht, t) - 1
    return i >= 0 and host[i][1] == 'autodrive'
def pct(v, q):
    if not v: return float('nan')
    v = sorted(v); return v[min(len(v) - 1, int(q * len(v)))]
ot = [o[0] for o in odom]
def speed(t):
    i = bisect.bisect_right(ot, t) - 1
    return odom[i][3] if i >= 0 else 0.0

dur = (odom[-1][0] - odom[0][0]) if len(odom) > 1 else 0
vmax = max((o[3] for o in odom), default=0)
print(f"== {a.bag}\n   span {dur:.0f} s | max speed {vmax:.2f} m/s | autodrive samples "
      f"{sum(1 for h in host if h[1]=='autodrive')}/{len(host)}")

# Step 51: use actual MPC optimizer/output layers, never infer optimal acceleration
# from a sparse VESC braking log. Report absence explicitly for old bags.
diag = [d for t, d in mpc_diagnostics if is_auto(t) and
        isinstance(d.get('optimal_accel_mps2'), (int, float))]
if diag:
    for key in ('optimal_accel_mps2', 'applied_accel_mps2', 'predicted_first_mps',
                'predicted_peak_mps', 'command_unclipped_mps', 'command_clipped_mps'):
        values = [d[key] for d in diag if isinstance(d.get(key), (int, float))]
        print(f'[MPC LAYERS] {key}: p50={pct(values,.5):.3f} p95={pct(values,.95):.3f}')
    for key in ('lead_limited', 'jerk_limited', 'plan_limited'):
        print(f'[MPC LAYERS] {key}: {100*sum(bool(d.get(key)) for d in diag)/len(diag):.1f}% of solved autodrive samples')
else:
    print('[MPC LAYERS] no solved autodrive diagnostics; optimizer bottleneck cannot be identified from this bag')

# --- tracking (frenet d = lateral offset from raceline)
d = [abs(f[2]) for f in fren if is_auto(f[0]) and speed(f[0]) > 0.5]
dist = 0.0
for p, q in zip(odom, odom[1:]):
    if is_auto(q[0]) and q[3] > 0.3: dist += math.hypot(q[1]-p[1], q[2]-p[2])
print(f"[TRACK] n={len(d)} dist {dist:.1f} m | |d| mean {sum(d)/len(d) if d else float('nan'):.3f} "
      f"p95 {pct(d,.95):.3f} max {max(d, default=float('nan')):.3f} m")
vs = [(c[1], speed(c[0])) for c in ctl if is_auto(c[0]) and c[1] > 0.5]
if vs:
    err = [abs(c - v) for c, v in vs]
    print(f"[SPEED] cmd max {max(c for c,_ in vs):.2f} | |cmd-actual| mean {sum(err)/len(err):.2f} p95 {pct(err,.95):.2f} m/s")

# --- steering stability (controller output during autodrive moving)
s = [(c[0], c[2]) for c in out if is_auto(c[0]) and speed(c[0]) > 0.5]
if len(s) > 2:
    rate = [abs((b[1]-a_[1])/(b[0]-a_[0])) for a_, b in zip(s, s[1:]) if b[0] > a_[0]]
    sat = sum(1 for _, x in s if abs(x) >= 0.395) / len(s)
    flips, last = 0, 0
    for _, x in s:
        sg = (x > 0.02) - (x < -0.02)
        if sg and last and sg != last: flips += 1
        if sg: last = sg
    span = s[-1][0] - s[0][0]
    yaw = [abs(o[4]) for o in odom if is_auto(o[0]) and o[3] > 0.5]
    print(f"[STEER] |steer| max {max(abs(x) for _,x in s):.3f} rad | sat(>=0.395) {sat*100:.1f}% | "
          f"rate p95 {pct(rate,.95):.2f} rad/s | sign flips {flips} ({flips/span*60 if span else 0:.0f}/min) | "
          f"yaw-rate p95 {pct(yaw,.95):.2f} rad/s")

# --- braking events (step 35: v0 threshold was a fixed 1.0 m/s -> 0 events at lowspeed 1.0)
BRAKE_V0 = 0.5
ev, i = [], 0
while i < len(out):
    t, v_cmd, _ = out[i]
    v0 = speed(t)
    if v_cmd <= 0.05 and v0 > BRAKE_V0 and (i == 0 or out[i-1][1] > 0.05):
        j = bisect.bisect_left(ot, t)
        k = j
        while k < len(odom) and odom[k][3] > 0.1: k += 1
        # Only full stops: the setpoint must stay ~0 until the car stops (a soft-STOP ramp
        # that re-accelerates is not a braking measurement).
        held = all(c_[1] <= 0.05 for c_ in out[i:] if c_[0] <= (odom[k][0] if k < len(odom) else t))
        if k < len(odom) and held:
            dd = sum(math.hypot(odom[n+1][1]-odom[n][1], odom[n+1][2]-odom[n][2]) for n in range(j, k))
            tt = odom[k][0] - t
            cause = (at(estop, t, (0, '?'))[1], at(host, t, (0, '?'))[1], at(st, t, (0, '?'))[1])
            ev.append((t - odom[0][0], v0, dd, tt, v0/tt if tt > 0 else 0, cause))
    i += 1
print(f"[BRAKE] events {len(ev)}  (t, v0, dist, time, mean decel, estop/host/mpc at cmd drop)")
for e in ev:
    print(f"   t={e[0]:6.1f}s v0 {e[1]:.2f} -> {e[2]:.2f} m in {e[3]:.2f} s ({e[4]:.2f} m/s^2)  {e[5]}")
if ev:
    dec = sorted(e[4] for e in ev)[len(ev)//2]
    for vn in (3.0, 4.0, 5.0):
        print(f"   predicted stop from {vn:.0f} m/s at median decel {dec:.2f}: {vn*vn/(2*dec):.2f} m")

# --- MPC
# Step 35: statuses are solved*/plan_continuation*/braking_recovery* (success) vs STOP
# reasons. The old filter (not startswith 'tracking') counted every message as non-tracking.
OK = ('solved', 'plan_continuation', 'braking_recovery')
mov = [(t, x) for t, x in st if is_auto(t) and speed(t) > 0.3]
bad = [x for _, x in mov if not x.startswith(OK)]
reasons = {}
for x in bad: reasons[x.split(':')[0]] = reasons.get(x.split(':')[0], 0) + 1
el = [x for t, x in el if is_auto(t) and speed(t) > 0.3]   # autodrive moving only
print(f"[MPC] autodrive moving ticks {len(mov)} | STOP {len(bad)} {reasons} | elapsed p50 {pct(el,.5):.1f} "
      f"p99 {pct(el,.99):.1f} max {max(el, default=float('nan')):.1f} ms")

# --- localisation consistency (step 35): |d| comes from the PF pose. Pose speed over
# 0.2 s vs wheel speed; outside [0.4, 2.0]x = pose stalled/jumped (|d| unreliable there).
W = 10
badpose = set()
for k in range(len(odom) - W):
    p, q = odom[k], odom[k + W]
    vw = 0.5 * (p[3] + q[3])
    if vw > 0.5 and q[0] > p[0]:
        r = math.hypot(q[1]-p[1], q[2]-p[2]) / (q[0]-p[0]) / vw
        if r < 0.4 or r > 2.0: badpose.add(k + W // 2)
def pose_ok(t):
    return (bisect.bisect_right(ot, t) - 1) not in badpose
dok = [abs(f[2]) for f in fren if is_auto(f[0]) and speed(f[0]) > 0.5 and pose_ok(f[0])]
print(f"[POSE] inconsistent pose {100*(1-len(dok)/len(d)) if d else 0:.0f}% of tracking samples | "
      f"|d| (pose ok) mean {sum(dok)/len(dok) if dok else float('nan'):.3f} p95 {pct(dok,.95):.3f}")

# --- go/no-go suggestion
lim = a.vmax
chk = []
if d: chk.append(('|d| p95 < 0.25 m', pct(d, .95) < 0.25)); chk.append(('|d| max < 0.50 m', max(d) < 0.5))
if len(s) > 2: chk.append(('steer sat < 5%', sat < 0.05)); chk.append(('steer flips < 60/min', flips/span*60 < 60 if span else True))
chk.append(('braking event measured', bool(ev)))
if el: chk.append(('MPC p99 < 16 ms', pct(el, .99) < 16))
if lim: chk.append((f'reached >= 90% of {lim} m/s', vmax >= 0.9*lim))
print("[GO?] " + " | ".join(f"{k}: {'OK' if v else 'NG'}" for k, v in chk))
