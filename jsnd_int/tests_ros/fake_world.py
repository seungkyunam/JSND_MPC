"""Scripted closed-loop ROS check for controller_manager(MPC) -> simple_mux (no VESC, no sensors).

Publishes the inputs the real stack would (global/local waypoints, odom, scan, joy) from a
kinematic car driven by the simple_mux OUTPUT, and records every command. Run in an isolated
ROS domain only.
"""
import json
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from f110_msgs.msg import BehaviorStrategy, Wpnt, WpntArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

MAP = Path.home() / 'JSND_INT/src/racing_stack/stack_master/maps/11_18_6/global_waypoints.json'
WB, GAIN, MIN_ERPM, TAU, DT = 0.324, 4614.0, 900.0, 0.15, 0.01

IN = lambda t, iv: any(a <= t < b for a, b in iv)
SCENARIOS = {
    # safety ordering while MPC drives; controller killed at the end
    'safety': dict(lifted=True, duration=27.0, rb=[(3.0, 3.2), (11.2, 11.4)], lb=[(10.0, 11.0)], odom_loss=[(13.0, 13.6)],
                   obstacle=[(15.0, 16.0)], beh_loss=[(17.0, 18.0)], trailing=[(19.5, 20.5)], ftg=[], kill=23.0,
                   phases=[(0.0, 3.0, 'no_host', 'mux publishes nothing'),
                           (3.0, 10.0, 'auto_mpc', 'MPC speed+steer, car moves'),
                           (10.0, 11.0, 'LB_human', 'mux 0 (human, stick neutral)'),
                           (11.2, 13.0, 'auto_again', 'MPC resumes'),
                           (13.0, 13.6, 'odom_lost', 'controller 0'),
                           (13.6, 15.0, 'odom_back', 'MPC resumes'),
                           (15.0, 16.0, 'scan_obstacle', 'mux estop 0 while controller > 0'),
                           (16.0, 17.0, 'clear', 'MPC'),
                           (17.0, 18.0, 'behavior_lost', 'controller 0 (waypoint safety)'),
                           (18.0, 19.5, 'behavior_back', 'MPC'),
                           (19.5, 20.5, 'TRAILING', 'controller 0 (unsupported)'),
                           (20.5, 23.0, 'GB_TRACK', 'MPC resumes'),
                           (23.0, 27.0, 'controller_killed', 'mux 0 after freshness 0.25 s')]),
    # FTGONLY runs the existing FTG in MPC mode
    'ftg': dict(duration=12.0, rb=[(3.0, 3.2)], lb=[], odom_loss=[], obstacle=[], beh_loss=[], trailing=[],
                ftg=[(6.0, 7.0)], kill=None,
                phases=[(3.0, 6.0, 'auto_mpc', 'MPC'),
                        (6.0, 7.0, 'FTGONLY', 'FTG command (ftg_max_speed 1.0)'),
                        (7.0, 12.0, 'GB_TRACK', 'MPC resumes or STOP if off path')]),
    # simple_mux safety_mode (latched E-stop + restart approval), car on the ground (not lifted)
    'estop': dict(duration=25.0, rb=[(3.0, 3.3), (5.0, 7.0), (7.5, 9.5), (12.0, 14.0), (16.0, 18.0), (20.5, 23.0)],
                  lb=[(9.0, 9.6), (12.8, 13.6)], a=[(13.0, 13.2)], stick_forward=[(9.0, 9.6), (12.8, 13.6)],
                  stick_value=0.25, joy_silent=[(16.8, 17.6)],
                  soft_estop=[21.5], resets=[4.0, 9.6, 11.6, 15.5, 19.8],
                  odom_loss=[], obstacle=[], beh_loss=[], trailing=[], ftg=[], kill=None,
                  events=[('startup latched', 0.0, 'none'), ('RB while latched', 3.0, 'none'),
                          ('RB hold', 5.0, 'drive'), ('RB released', 7.0, 'release'), ('RB hold again', 7.5, 'drive'),
                          ('LB + stick 0.25 (manual)', 9.0, 'drive'), ('LB released', 9.6, 'release'),
                          ('LB manual then A', 12.8, 'drive'), ('A button (LB held)', 13.0, 'latch'),
                          ('joy lost', 16.8, 'latch'), ('soft estop topic', 21.5, 'latch')],
                  phases=[]),
    # 2026-11-22 step 12: LB manual with the stick fully forward -> MPC mode caps at joy_max_speed 1.0 m/s
    'lbmax': None,
}
SCENARIOS['lbmax'] = dict(SCENARIOS['estop'], stick_value=1.0, events=[
    (('LB + stick 1.0 (manual, full)' if n.startswith('LB + stick') else n), t, k) for n, t, k in SCENARIOS['estop']['events']])
SC = SCENARIOS[sys.argv[1] if len(sys.argv) > 1 else 'safety']
PHASES = SC['phases']


class World(Node):
    def __init__(self):
        super().__init__('fake_world')
        w = json.loads(MAP.read_text())['global_traj_wpnts_iqp']['wpnts']
        self.line = w
        self.xy = np.array([[p['x_m'], p['y_m']] for p in w])
        self.x, self.y, self.yaw = w[0]['x_m'], w[0]['y_m'], w[0]['psi_rad']
        self.v = 0.0
        self.cmd = None
        self.t0 = time.monotonic()
        self.log = defaultdict(list)
        self.killed = False
        self.pub_glob = self.create_publisher(WpntArray, '/global_waypoints', 10)
        self.pub_beh = self.create_publisher(BehaviorStrategy, '/behavior_strategy', 10)
        self.pub_odom = self.create_publisher(Odometry, '/car_state/odom', 10)
        self.pub_fren = self.create_publisher(Odometry, '/car_state/odom_frenet', 10)
        self.pub_vodom = self.create_publisher(Odometry, '/vesc/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_joy = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(AckermannDriveStamped, '/vesc/ackermann_cmd', self.mux_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd', self.ctrl_cb, 10)
        self.create_subscription(String, '/controller/mpc_status', lambda m: self.rec('status', m.data), 10)
        self.create_subscription(Float32, '/controller/mpc_elapsed_ms', lambda m: self.rec('elapsed', m.data), 10)
        self.create_timer(DT, self.physics)
        self.create_timer(0.02, self.pub_odoms)
        self.create_timer(0.025, self.pub_behavior)
        self.create_timer(0.025, self.pub_lidar)
        self.create_timer(0.05, self.pub_joystick)
        self.create_timer(1.0, self.pub_global)
        self.create_subscription(String, '/simple_mux/active_host', lambda m: self.rec('host', m.data), 10)
        self.create_subscription(String, '/simple_mux/estop_state', lambda m: self.rec('estop_state', m.data), 10)
        self.create_subscription(Float32, '/controller/mpc_loop_period_ms', lambda m: self.rec('period', m.data), 10)
        self.create_subscription(Float32, '/controller/mpc_cycle_ms', lambda m: self.rec('cycle', m.data), 10)
        self.pub_soft = self.create_publisher(Bool, '/simple_mux/estop', 10)
        self.reset_cli = self.create_client(Trigger, '/simple_mux/reset_estop')
        self.pending = list(SC.get('resets', []))
        self.soft = list(SC.get('soft_estop', []))
        self.reset_futures = []

    def t(self):
        return time.monotonic() - self.t0

    def rec(self, key, value):
        self.log[key].append((self.t(), value))

    def mux_cb(self, m):
        self.cmd = m
        self.rec('mux', (m.drive.speed, m.drive.steering_angle))

    def ctrl_cb(self, m):
        self.rec('ctrl', (m.drive.speed, m.drive.steering_angle, m.drive.acceleration))

    def physics(self):
        speed = self.cmd.drive.speed if self.cmd else 0.0
        steer = self.cmd.drive.steering_angle if self.cmd else 0.0
        target = speed if abs(speed * GAIN) >= MIN_ERPM else 0.0
        self.v += (target - self.v) * DT / TAU
        self.rec('steer_cmd', steer)
        if not SC.get('lifted'):  # lifted: wheels spin, chassis pose fixed
            self.x += self.v * np.cos(self.yaw) * DT
            self.y += self.v * np.sin(self.yaw) * DT
            self.yaw += self.v * np.tan(steer) / WB * DT
        self.rec('v', self.v)

    def pub_odoms(self):
        t = self.t()
        if IN(t, SC['odom_loss']):
            return  # odom loss
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = 'map'
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.x), float(self.y)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = float(np.sin(self.yaw / 2)), float(np.cos(self.yaw / 2))
        o.twist.twist.linear.x = float(self.v)
        self.pub_odom.publish(o)
        self.pub_vodom.publish(o)
        f = Odometry()
        f.header.stamp = now
        i = self.nearest()
        f.pose.pose.position.x = float(self.line[i]['s_m'])
        f.twist.twist.linear.x = float(self.v)
        self.pub_fren.publish(f)

    def nearest(self):
        return int(np.argmin(np.linalg.norm(self.xy - [self.x, self.y], axis=1)))

    def wpnt(self, p):
        w = Wpnt()
        for k in ('id',):
            setattr(w, k, int(p[k]))
        for k in ('s_m', 'd_m', 'x_m', 'y_m', 'd_right', 'd_left', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'):
            setattr(w, k, float(p[k]))
        return w

    def pub_global(self):
        m = WpntArray()
        m.header.frame_id = 'map'
        m.wpnts = [self.wpnt(p) for p in self.line]
        self.pub_glob.publish(m)

    def pub_behavior(self):
        t = self.t()
        if IN(t, SC['beh_loss']):
            return  # behavior_strategy loss
        state = 'FTGONLY' if IN(t, SC['ftg']) else 'TRAILING' if IN(t, SC['trailing']) else 'GB_TRACK'
        i = self.nearest()
        m = BehaviorStrategy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = state
        m.local_wpnts = [self.wpnt(self.line[(i + k) % len(self.line)]) for k in range(80)]
        self.pub_beh.publish(m)

    def pub_lidar(self):
        t = self.t()
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'laser'
        s.angle_min, s.angle_max = -2.35619, 2.35619
        n = 1081
        s.angle_increment = (s.angle_max - s.angle_min) / (n - 1)
        s.range_min, s.range_max = 0.02, 30.0
        r = np.full(n, 5.0, dtype=np.float32)
        if IN(t, SC['obstacle']):
            r[n // 2 - 40:n // 2 + 40] = 0.15  # obstacle right in front
        s.ranges = r.tolist()
        self.pub_scan.publish(s)

    def pub_joystick(self):
        t = self.t()
        j = Joy()
        j.header.stamp = self.get_clock().now().to_msg()
        j.axes = [0.0] * 8
        j.buttons = [0] * 11
        if IN(t, SC['rb']):
            j.buttons[5] = 1  # RB -> autodrive
        if IN(t, SC['lb']):
            j.buttons[4] = 1  # LB held, sticks neutral -> human 0
        if IN(t, SC.get('a', [])):
            j.buttons[0] = 1  # A
        if IN(t, SC.get('stick_forward', [])):
            j.axes[1] = SC.get('stick_value', 1.0)   # forward stick (1.0 = 2026-11-22 18456 ERPM incident)
        if self.pending and t >= self.pending[0]:
            at = self.pending.pop(0)
            fut = self.reset_cli.call_async(Trigger.Request())
            self.reset_futures.append((at, fut))
        if self.soft and t >= self.soft[0]:
            self.soft.pop(0)
            self.pub_soft.publish(Bool(data=True))
            self.rec('soft', 1)
        if IN(t, SC.get('joy_silent', [])):
            return            # pad lost: no /joy at all
        self.pub_joy.publish(j)
        if SC['kill'] is not None and t >= SC['kill'] and not self.killed:
            self.killed = True
            subprocess.run(['pkill', '-INT', '-f', 'lib/controller/controller_manager'], check=False)
            self.rec('killed', 1)


def summarize_estop(node):
    log = node.log
    mux, host = log['mux'], log['host']
    def first(seq, t0, pred):
        return next((t for t, v in seq if t >= t0 and pred(v)), None)
    print('reset requests:')
    for at, fut in node.reset_futures:
        r = fut.result() if fut.done() else None
        host_after = next((v for t, v in log['host'] if t > at + 0.05), None)
        print(f'  t={at:5.1f}s  ' + (f'success={r.success}  {r.message}' if r else 'NO RESPONSE') + f'  [host after: {host_after}]')
    print(f"{'event':26s} {'t':>5s}  result")
    for name, t0, kind in SC['events']:
        nxt = min([e[1] for e in SC['events'] if e[1] > t0] + [SC['duration']])
        seg = [v for t, v in mux if t0 <= t < nxt]
        hosts = sorted({v for t, v in host if t0 <= t < nxt})
        vmax = max([v[0] for v in seg], default=float('nan'))
        if kind == 'latch':
            th = first(host, t0, lambda v: v == 'estop')
            until = min([r for r in SC['resets'] if r > t0] + [SC['duration']])
            latched = [v for t, v in mux if th is not None and th + 0.02 <= t < until]   # one mux tick after the latch
            ok = [r.success for at, fut in node.reset_futures if at > t0 and fut.done() for r in [fut.result()]]
            carv = [(t, v) for t, v in log['v'] if th is not None and t >= th]
            stop_t = next((t for t, v in carv if v < 0.05), None)
            v_at = max([v for t, v in log['v'] if th is not None and abs(t - th) < 0.01], default=float('nan'))
            print(f'{name:26s} {t0:5.1f}  host->estop +{(th - t0) if th else float("nan"):.3f}s; until next reset ({until:.1f}s) '
                  f'mux n={len(latched)} speed max {max([v[0] for v in latched], default=float("nan")):.3f} '
                  f'|steer| max {max([abs(v[1]) for v in latched], default=float("nan")):.3f}; car {v_at:.2f} m/s -> <0.05 in '
                  f'{(stop_t - th) if stop_t else float("nan"):.2f}s (model tau {TAU}s)')
        else:
            print(f'{name:26s} {t0:5.1f}  mux n={len(seg)} speed max {vmax:.3f}, hosts {hosts}')
    states = [v for t, v in log['estop_state']]
    print('estop_state changes:', [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]])
    mt = np.array([t for t, _ in mux])
    print(f'mux output rate {len(mt) / (mt[-1] - mt[0]):.1f} Hz, period p50 {1e3*np.median(np.diff(mt)):.1f} p99 {1e3*np.percentile(np.diff(mt), 99):.1f} ms; '
          f'max mux speed overall {max(v[0] for _, v in mux):.3f} m/s; humandrive host seen: {any(v == "humandrive" for _, v in host)}')
    rates(log)


def rates(log):
    ct = np.array([t for t, _ in log['ctrl']])
    per = np.array([v for _, v in log['period']]) if log['period'] else np.zeros(1)
    cyc = np.array([v for _, v in log['cycle']]) if log['cycle'] else np.zeros(1)
    el = np.array([v for _, v in log['elapsed']]) if log['elapsed'] else np.zeros(1)
    solved = [t for t, v in log['status'] if v == 'solved']
    span = (ct[-1] - ct[0]) if len(ct) > 1 else 1.0
    print(f'OUTPUT  controller publish {len(ct) / span:.1f} Hz (receive-side period p50 {1e3*np.median(np.diff(ct)):.1f} p99 {1e3*np.percentile(np.diff(ct), 99):.1f} max {1e3*np.diff(ct).max():.1f} ms); '
          f'timer period (node) p50 {np.median(per):.2f} p99 {np.percentile(per, 99):.2f} max {per.max():.2f} ms')
    print(f'COMPUTE control cycle ms p50 {np.median(cyc):.2f} p99 {np.percentile(cyc, 99):.2f} max {cyc.max():.2f}; '
          f'MPC solve ms p50 {np.median(el):.2f} p99 {np.percentile(el, 99):.2f} max {el.max():.2f}; '
          f'solved solves/s {len(solved) / span:.1f} ({len(solved)}/{len(log["status"])} ticks)')


def summarize(log):
    print(f"{'phase':18s} {'expect':38s} {'ctrl n':>6s} {'ctrl v(mean/max)':>17s} {'ctrl |steer|max':>15s} "
          f"{'mux n':>5s} {'mux v max':>9s} {'car v max':>9s}  status")
    for a, b, name, expect in PHASES:
        a2 = a + 0.3 if name not in ('no_host',) else a
        if name == 'controller_killed':
            a2 = a + 1.3  # freshness 1.0 s + margin
        c = [v for t, v in log['ctrl'] if a2 <= t < b]
        m = [v for t, v in log['mux'] if a2 <= t < b]
        car = [v for t, v in log['v'] if a2 <= t < b]
        st = sorted({v for t, v in log['status'] if a2 <= t < b})
        cv = np.array([x[0] for x in c]) if c else np.zeros(1)
        cs = np.array([abs(x[1]) for x in c]) if c else np.zeros(1)
        mv = np.array([x[0] for x in m]) if m else np.zeros(1)
        print(f"{name:18s} {expect:38s} {len(c):6d} {cv.mean():8.3f}/{cv.max():7.3f} {cs.max():15.3f} "
              f"{len(m):5d} {mv.max():9.3f} {max(car) if car else 0:9.3f}  {st}")
    ct = [t for t, _ in log['ctrl'] if 3 < t < (SC['kill'] or SC['duration'])]
    el = np.array([v for _, v in log['elapsed']]) if log['elapsed'] else np.zeros(1)   # PP: no MPC timing
    print(f"controller rate {len(ct) / (ct[-1] - ct[0]):.1f} Hz; period p99 {1e3*np.percentile(np.diff(ct), 99):.1f} ms "
          f"max {1e3*np.max(np.diff(ct)):.1f} ms; mpc elapsed ms p50 {np.median(el):.2f} p99 {np.percentile(el, 99):.2f} max {el.max():.2f}")
    # engage: controller speed before RB, first commands after RB
    rb = SC['rb'][0][0]
    pre = [v[0] for t, v in log['ctrl'] if t < rb]
    post = [(t, v[0]) for t, v in log['mux'] if t >= rb][:8]
    print(f"ctrl speed before RB: max {max(pre) if pre else 0:.3f} m/s; first mux after RB: "
          + ', '.join(f'{v:.3f}' for _, v in post))
    if SC['kill'] is not None:
        k = SC['kill']
        last_move = [t for t, v in log['mux'] if t < k and v[0] > 0]
        zero = [t for t, v in log['mux'] if t >= k and v[0] == 0.0]
        nz_after = [t for t, v in log['mux'] if t >= k and v[0] > 0]
        print(f"kill at {k:.2f}s: mux speed before kill {'>0' if last_move and k - last_move[-1] < 0.1 else 'NOT moving'}; "
              f"last nonzero mux {max(nz_after) - k if nz_after else 0:.3f}s after kill; "
              f"first zero {zero[0] - k if zero else float('nan'):.3f}s after kill")
    start = [t for t, v in log['mux'] if v[0] * GAIN >= MIN_ERPM]
    print(f"first mux speed >= {MIN_ERPM:.0f} ERPM at t={start[0]:.2f}s" if start else 'never reached min ERPM')


def main():
    rclpy.init()
    node = World()
    end = time.monotonic() + SC['duration']
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.005)
    if sys.argv[1:2] in (['estop'], ['lbmax']):
        summarize_estop(node)
    else:
        summarize(node.log)
        if node.log['period']:
            rates(node.log)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
