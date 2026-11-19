#!/usr/bin/env python3
"""Read-only isolated ROS simulation lap recorder. Never publishes commands.

Start before enabling simulated joystick AUTO. Requires a running MPC simulation.
Collision clearance is not inferred from odometry; review simulator collision logs.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from sim_probe import launch_environment


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--map-dir', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--timeout', type=float, default=300.)
    args = ap.parse_args()
    launch_environment(os.environ)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ackermann_msgs.msg import AckermannDriveStamped
    from rcl_interfaces.srv import GetParameters
    import sim_probe
    sim_probe.rclpy, sim_probe.GetParameters = rclpy, GetParameters
    source = args.map_dir/'global_waypoints.json'
    wp = json.loads(source.read_text())['global_traj_wpnts_iqp']['wpnts']
    path = np.array([[p['x_m'], p['y_m']] for p in wp])
    if np.linalg.norm(path[0]-path[-1]) < 1e-6: path = path[:-1]
    segments = np.roll(path, -1, axis=0)-path
    lengths = np.linalg.norm(segments, axis=1)
    if len(path) < 20 or not np.isfinite(path).all() or np.any(lengths < 1e-6):
        raise ValueError('invalid closed global path')
    arc = np.r_[0., np.cumsum(lengths)]
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('integrated_mpc_lap_recorder')
    rows, laps, failures = [], [], {}
    latest = {'command': (0., 0.), 'status': 'waiting', 'odom': None}
    def odom(msg): latest['odom'] = msg
    def command(msg): latest['command'] = (msg.drive.speed, msg.drive.steering_angle)
    def status(msg): latest['status'] = msg.data
    node.create_subscription(Odometry, '/car_state/odom', odom, qos_profile_sensor_data)
    node.create_subscription(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd', command, 10)
    node.create_subscription(String, '/controller/mpc_status', status, 10)
    start = time.monotonic(); moving = False; previous_s = None; progress = 0.
    last_stamp = None; lap_start = None; error = None
    try:
        params = sim_probe.parameters(node, 'controller_manager', ['sim', 'lateral_controller'])
        if params != {'sim': True, 'lateral_controller': 'MPC'}:
            raise RuntimeError('requires integrated MPC simulation manager')
        selected = sim_probe.parameters(node, 'global_republisher', ['map'])
        if selected.get('map') != args.map_dir.name:
            raise RuntimeError('selected simulation map does not match recording path')
        while time.monotonic()-start < args.timeout and len(laps) < 3:
            rclpy.spin_once(node, timeout_sec=.05)
            msg = latest['odom']
            if msg is None: continue
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp == last_stamp: continue
            last_stamp = stamp
            if msg.header.frame_id != 'map': raise RuntimeError('odometry must be in map')
            xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            fraction = np.clip(np.sum((xy-path)*segments, axis=1)/lengths**2, 0., 1.)
            distances = np.linalg.norm(xy-path-fraction[:, None]*segments, axis=1)
            idx = int(np.argmin(distances)); s = arc[idx]+fraction[idx]*lengths[idx]
            v = msg.twist.twist.linear.x
            if not moving and v > .05:
                moving = True; previous_s = s; lap_start = stamp
            if not moving: continue
            advance = (s-previous_s+arc[-1]/2)%arc[-1]-arc[-1]/2
            if abs(advance) > 1.: raise RuntimeError('pose jump or recording gap')
            progress += advance; previous_s = s
            if progress >= (len(laps)+1)*arc[-1]:
                laps.append(stamp-lap_start); lap_start = stamp
            rows.append((stamp, *xy, v, *latest['command'], distances[idx], s, latest['status']))
            if latest['status'] != 'solved':
                failures[latest['status']] = failures.get(latest['status'], 0)+1
            if distances[idx] > 1.: raise RuntimeError('tracking envelope exceeded')
    except Exception as exc:
        error = str(exc)
    finally:
        node.destroy_node(); rclpy.shutdown()
    errors = np.array([row[6] for row in rows])
    result = dict(kind='ROS_odometry_observation', map=str(args.map_dir),
                  path_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  laps_s=laps, solver_fault_samples=failures, error=error,
                  rmse_m=float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
                  max_error_m=float(np.max(errors)) if len(errors) else None,
                  three_laps_without_solver_fault=len(laps)==3 and not failures and error is None,
                  collision_clearance_verified=False)
    with (args.output/'trajectory.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(['stamp','x','y','speed','command_speed','steering','error','arc','status']); writer.writerows(rows)
    (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['three_laps_without_solver_fault'] else 1


if __name__ == '__main__': raise SystemExit(main())
