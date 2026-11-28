#!/usr/bin/env python3
"""Estimate the real VESC first-order speed-response constant from an MCAP bag.

Uses post-mux /vesc/ackermann_cmd as v_cmd and /car_state/odom as measured v.
Only fresh autodrive samples with command error and acceleration in the same
direction are fitted. This is read-only and never publishes a vehicle command.
"""

import argparse
import bisect
import json
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def previous(seq, stamp):
    times = [row[0] for row in seq]
    index = bisect.bisect_right(times, stamp) - 1
    return seq[index] if index >= 0 else None


def fit(samples):
    if len(samples) < 30:
        return None
    data = np.asarray(samples, dtype=float)
    x, y = data[:, 0], data[:, 1]
    keep = np.ones(len(x), dtype=bool)
    for _ in range(3):
        xx, yy = x[keep], y[keep]
        gain = float(np.dot(xx, yy) / np.dot(xx, xx))
        residual = yy - gain * xx
        scale = 1.4826 * np.median(np.abs(residual - np.median(residual)))
        if not math.isfinite(scale) or scale < 1e-6:
            break
        current = np.flatnonzero(keep)
        keep[current[np.abs(residual) > 3.5 * scale]] = False
    xx, yy = x[keep], y[keep]
    gain = float(np.dot(xx, yy) / np.dot(xx, xx))
    tau = 1.0 / gain if gain > 0 else float('nan')
    predicted = gain * xx
    denom = float(np.sum((yy - np.mean(yy)) ** 2))
    r2 = 1.0 - float(np.sum((yy - predicted) ** 2)) / denom if denom > 0 else float('nan')
    point_tau = xx / yy
    point_tau = point_tau[np.isfinite(point_tau) & (point_tau > 0.) & (point_tau < 1.)]
    return dict(n=int(len(xx)), tau_s=tau, r2=r2,
                point_tau_p50_s=float(np.median(point_tau)) if len(point_tau) else None,
                point_tau_p10_s=float(np.percentile(point_tau, 10)) if len(point_tau) else None,
                point_tau_p90_s=float(np.percentile(point_tau, 90)) if len(point_tau) else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='completed rosbag2 MCAP directory')
    parser.add_argument('--window-s', type=float, default=.12,
                        help='central finite-difference window (default: 0.12 s)')
    parser.add_argument('--min-error-mps', type=float, default=.08)
    args = parser.parse_args()
    if not .04 <= args.window_s <= .5 or args.min_error_mps <= 0:
        parser.error('invalid fit window/error threshold')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    odom, commands, hosts, diagnostics = [], [], [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(raw, types[topic])
        t = stamp * 1e-9
        if topic == '/car_state/odom':
            odom.append((t, float(msg.twist.twist.linear.x)))
        elif topic == '/vesc/ackermann_cmd':
            commands.append((t, float(msg.drive.speed)))
        elif topic == '/simple_mux/active_host':
            hosts.append((t, str(msg.data)))
        elif topic == '/controller/mpc_diagnostics':
            try:
                diagnostics.append(json.loads(msg.data))
            except (TypeError, ValueError):
                pass
    if len(odom) < 50 or len(commands) < 20 or not hosts:
        raise SystemExit('bag lacks odom, post-mux command, or active-host samples')

    half = args.window_s / 2.
    samples = []
    for index, (t, speed) in enumerate(odom):
        left = bisect.bisect_left(odom, (t - half, -float('inf')))
        right = bisect.bisect_right(odom, (t + half, float('inf'))) - 1
        if left >= index or right <= index:
            continue
        dt = odom[right][0] - odom[left][0]
        if dt < .75 * args.window_s:
            continue
        accel = (odom[right][1] - odom[left][1]) / dt
        cmd = previous(commands, t)
        host = previous(hosts, t)
        if cmd is None or host is None or host[1] != 'autodrive' or t - cmd[0] > .08:
            continue
        error = cmd[1] - speed
        if (abs(error) < args.min_error_mps or abs(accel) < .1 or abs(accel) > 8.
                or error * accel <= 0 or not all(map(math.isfinite, (error, accel)))):
            continue
        samples.append((error, accel))

    configured = [d.get('response_tau_s') for d in diagnostics
                  if isinstance(d.get('response_tau_s'), (int, float))]
    report = {
        'bag': args.bag,
        'model': 'dv/dt = (v_cmd - v_measured) / tau',
        'configured_tau_s': float(np.median(configured)) if configured else None,
        'window_s': args.window_s,
        'all': fit(samples),
        'acceleration': fit([row for row in samples if row[0] > 0]),
        'braking': fit([row for row in samples if row[0] < 0]),
    }
    print(json.dumps(report, indent=2, allow_nan=False))
    if report['all'] is None:
        raise SystemExit('insufficient excitation: collect several safe accel/coast/brake steps')
    print('Interpretation: compare fitted tau with 0.12 s; inspect accel/brake split and R^2. '
          'Do not change the controller from one run without repeatable, physically reviewed evidence.')


if __name__ == '__main__':
    main()
