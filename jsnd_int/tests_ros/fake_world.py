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
from std_msgs.msg import Float32, String

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
}
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
        self.pub_joy.publish(j)
        if SC['kill'] is not None and t >= SC['kill'] and not self.killed:
            self.killed = True
            subprocess.run(['pkill', '-INT', '-f', 'lib/controller/controller_manager'], check=False)
            self.rec('killed', 1)


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
    el = np.array([v for _, v in log['elapsed']])
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
    summarize(node.log)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
