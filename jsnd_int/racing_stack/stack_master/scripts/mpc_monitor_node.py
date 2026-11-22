#!/usr/bin/env python3
"""MPC real-time performance monitor (JSND_INT, controller:=mpc only).

Separate process: subscribes to the topics controller_manager already publishes and
publishes ONE low-rate text summary (/pitwall/mpc_monitor, std_msgs/String, default 2 Hz)
for the RViz pitwall panel. It never publishes to the control path and does not touch
controller_manager, so the 100 Hz loop is unaffected (cost: a few cheap callbacks).

Measured inputs (no estimates; missing/stale data -> N/A):
  /controller/mpc_loop_period_ms  actual control period (timer tick to tick)
  /controller/mpc_cycle_ms        whole control cycle compute time (incl. MPC)
  /controller/mpc_elapsed_ms      MPC solve time only
  /controller/mpc_status          'solved' or failure reason
  /vesc/high_level/ackermann_cmd  controller target speed
  /car_state/odom                 measured speed
Set value: target_hz (= race.launch mpc_rate_hz, same value given to controller_manager).
"""
import collections
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String

NA = 'N/A'


class MpcMonitor(Node):
    def __init__(self):
        super().__init__('mpc_monitor')
        self.target_hz = float(self.declare_parameter('target_hz', 100.0).value)
        self.window_s = float(self.declare_parameter('window_s', 10.0).value)
        self.stale_s = float(self.declare_parameter('stale_s', 1.0).value)
        rate = float(self.declare_parameter('publish_hz', 2.0).value)
        self.target_ms = 1000.0 / self.target_hz if self.target_hz > 0 else None
        n = int(self.window_s * max(self.target_hz, 1.0) * 1.5) + 10
        self.period = collections.deque(maxlen=n)    # (t, ms)
        self.cycle = collections.deque(maxlen=n)
        self.solve = collections.deque(maxlen=n)
        self.status = collections.deque(maxlen=n)    # (t, ok)
        self.last = {}                               # key -> (t, value)
        self.overrun_cycle = 0                       # cycle compute > set period (cumulative)
        self.overrun_period = 0                      # actual period > 1.5 x set period (cumulative)
        self.solved_total = self.failed_total = 0
        self.last_fail = None
        f = self.create_subscription
        f(Float32, '/controller/mpc_loop_period_ms', self._period, 50)
        f(Float32, '/controller/mpc_cycle_ms', self._cycle, 50)
        f(Float32, '/controller/mpc_elapsed_ms', self._solve, 50)
        f(String, '/controller/mpc_status', self._status, 50)
        f(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd',
          lambda m: self._set('v_cmd', m.drive.speed), 10)
        f(Odometry, '/car_state/odom', lambda m: self._set('v_meas', m.twist.twist.linear.x),
          qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/pitwall/mpc_monitor', 1)
        self.create_timer(1.0 / rate, self._publish)

    def _set(self, key, value):
        self.last[key] = (time.monotonic(), value)

    def _period(self, m):
        t = time.monotonic()
        self.period.append((t, m.data))
        if self.target_ms and m.data > 1.5 * self.target_ms:
            self.overrun_period += 1

    def _cycle(self, m):
        t = time.monotonic()
        self.cycle.append((t, m.data))
        if self.target_ms and m.data > self.target_ms:
            self.overrun_cycle += 1

    def _solve(self, m):
        self.solve.append((time.monotonic(), m.data))

    def _status(self, m):
        ok = m.data == 'solved'
        self.status.append((time.monotonic(), ok))
        self._set('status', m.data)
        if ok:
            self.solved_total += 1
        else:
            self.failed_total += 1
            self.last_fail = m.data

    def _recent(self, buf, now):
        lo = now - self.window_s
        vals = [v for t, v in buf if t >= lo]
        if not buf or now - buf[-1][0] > self.stale_s or not vals:
            return None
        return np.asarray(vals, float)

    def _fresh(self, key, now):
        item = self.last.get(key)
        return None if item is None or now - item[0] > self.stale_s else item[1]

    def _publish(self):
        now = time.monotonic()
        w = f'{self.window_s:.0f}s'
        lines = []
        tgt = (f'{self.target_ms:.2f} ms / {self.target_hz:.0f} Hz' if self.target_ms else NA)
        lines.append(f'Set period    : {tgt}')
        p = self._recent(self.period, now)
        if p is None:
            lines.append('Actual period : N/A')
        else:
            mean = float(np.mean(p))
            lines.append(f'Actual period : {mean:.2f} ms / {1000.0 / mean:.1f} Hz '
                         f'(p99 {np.percentile(p, 99):.2f}, max {p.max():.2f} ms, {w})')
        c = self._recent(self.cycle, now)
        lines.append('Cycle compute : N/A' if c is None else
                     f'Cycle compute : {c[-1]:.2f} ms (p99 {np.percentile(c, 99):.2f}, '
                     f'max {c.max():.2f} ms, {w})')
        s = self._recent(self.solve, now)
        lines.append('MPC solve     : N/A' if s is None else
                     f'MPC solve     : {s[-1]:.2f} ms (p99 {np.percentile(s, 99):.2f}, '
                     f'max {s.max():.2f} ms, {w})')
        st = self._fresh('status', now)
        recent = [ok for t, ok in self.status if t >= now - self.window_s]
        if st is None:
            lines.append('OSQP status   : N/A')
        else:
            word = 'Solved' if st == 'solved' else f'Failed ({st})'
            lines.append(f'OSQP status   : {word}  [{w}: {sum(recent)} solved / '
                         f'{len(recent) - sum(recent)} failed]')
        lines.append(f'Totals        : solved {self.solved_total}, failed {self.failed_total}'
                     + (f', last fail: {self.last_fail}' if self.last_fail else ''))
        if self.target_ms:
            lines.append(f'Overruns      : compute>{self.target_ms:.0f}ms {self.overrun_cycle}, '
                         f'period>{1.5 * self.target_ms:.0f}ms {self.overrun_period} (since start)')
        else:
            lines.append('Overruns      : N/A')
        vc, vm = self._fresh('v_cmd', now), self._fresh('v_meas', now)
        fmt = lambda v: NA if v is None else f'{v:.2f}'
        lines.append(f'Speed m/s     : target {fmt(vc)} | actual {fmt(vm)}')
        self.pub.publish(String(data='\n'.join(lines)))


def main():
    rclpy.init()
    node = MpcMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
