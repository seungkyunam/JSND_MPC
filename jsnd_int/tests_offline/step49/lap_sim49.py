#!/usr/bin/env python3
"""Offline closed-loop lap simulation on 11_18_6: legacy PP vs integrated MPC with the SAME
speed plan (/global_waypoints iqp raceline x speed_scaling.yaml sectors -> GlobalTracking
local window, exactly as state_machine builds /local_waypoints in GB_TRACK).

No ROS graph, no hardware. The real Controller class (controller_manager's inner loop) is
driven; the vehicle is a rear-axle kinematic bicycle with a steering servo (rate limit +
1 tick delay) and a VESC speed loop (first-order, accel/decel saturation). These actuator
numbers are ASSUMPTIONS (not measured on the car) and identical for both controllers.

  source /opt/ros/jazzy/setup.bash; source ~/JSND_INT/install/setup.bash
  PYTHONPATH=~/JSND_INT/src/racing_stack/controller:~/JSND_INT/pydeps:$PYTHONPATH \
      python3 ~/JSND_INT/tests_offline/lap_sim.py [--ctrl pp|mpc|both] [--profile 5p0] [--set k=v ...]
"""
import argparse
import collections
import inspect
import json
from pathlib import Path

import numpy as np
import yaml

ROOT = Path.home() / 'JSND_INT/src/racing_stack/stack_master'
CFG = ROOT / 'config'
MAP = ROOT / 'maps/11_18_6'

# ---- actuator assumptions (same for PP and MPC) ----
SERVO_RATE = 6.0       # rad/s  (typical hobby servo ~0.1 s/60deg; NOT measured)
SERVO_LIMIT = 0.49     # rad    (servo 0..1 <=> +0.492/-0.518 rad, vehicle_config)
VESC_TAU = 0.12        # s      speed-loop first-order time constant (assumed)
VESC_ACC, VESC_DEC = 4.0, 6.0   # m/s^2 saturation (raceline ax spans -3.2..+3.6)
REAL_DEADLINES = False
HISTORY = False
BIAS = 0.0
SIMT = [0.0]


def params(path):
    return yaml.safe_load(Path(path).read_text())['controller_manager']['ros__parameters']


def load_track():
    d = json.loads((MAP / 'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    sc = yaml.safe_load((MAP / 'speed_scaling.yaml').read_text())['speed_sector_tuner']['ros__parameters']
    scale = np.ones(len(d))
    for i in range(sc['n_sectors']):
        s = sc[f'Sector{i}']
        scale[s['start']:s['end'] + 1] = s['scaling'] * sc.get('phase_multiplier', 1.0)
    rows = []
    for p, k in zip(d, scale):
        dl, dr = p['d_left'], p['d_right']
        rows.append([p['x_m'], p['y_m'], p['vx_mps'] * k, min(dl, dr) / (dl + dr) if dl + dr else 0,
                     p['s_m'], p['kappa_radpm'], p['psi_rad'], p['ax_mps2'], p['d_m']])
    full = np.asarray(rows, float)
    return full, full[:-1]          # state_machine drops the closing point


FULL, GB = load_track()
TRACK_LEN = FULL[-1, 4]
WP_DIST = GB[1, 4] - GB[0, 4]


def local_wpnts(s, n=80):
    i = int(s / WP_DIST + 0.5)
    return GB[(np.arange(n) + i) % len(GB)]


def make_converter():
    from frenet_conversion.frenet_converter import FrenetConverter
    return FrenetConverter(FULL[:, 0], FULL[:, 1])


def build(ctrl, profile, overrides, rate):
    from controller.combined.src.Controller import Controller
    p = params(CFG / 'controller.yaml')
    p.update(params(CFG / 'dynamic_controller.yaml'))
    lateral_mpc = None
    if ctrl == 'mpc':
        from controller.mpc.configuration import MPC_DEFAULTS, make_config, make_steering_config
        from controller.mpc.integrated_mpc import IntegratedMPC
        from controller.mpc.steering_mpc import SteeringMPC
        p.update(params(CFG / 'mpc_car.yaml'))
        p.update(params(CFG / f'mpc_profile_{profile}.yaml'))
        p.update({'mpc_deadline_s': .8 / rate, 'mpc_solver_time_limit_s': .4 / rate})  # race.launch
        p.update(overrides)
        vals = {k: p.get(k, v) for k, v in MPC_DEFAULTS.items()}
        # offline: do not let Jetson-independent wall-clock jitter decide the result
        # (REAL_DEADLINES=True keeps race.launch's 0.8/0.4 x period budgets)
        if not REAL_DEADLINES:
            vals['mpc_deadline_s'] = 1.0 / rate
            vals['mpc_solver_time_limit_s'] = 1.0 / rate
        if vals['mpc_speed_source'] == 'jsnd':
            lateral_mpc = SteeringMPC(make_steering_config(vals, p['wheelbase'], True, rate))
        else:
            lateral_mpc = IntegratedMPC(make_config(vals, p['wheelbase'], True, rate))
        p['_mpc_vals'] = vals
    else:
        p.update(overrides)
    sig = inspect.signature(Controller.__init__)
    kw = {}
    for name, prm in sig.parameters.items():
        if name == 'self' or prm.default is not inspect.Parameter.empty:
            continue
        kw[name] = p.get(name, 0.0)
    kw.update(loop_rate=rate, wheelbase=p.get('wheelbase', 0.324), converter=make_converter())
    extra = dict(lateral_controller='MPC' if ctrl == 'mpc' else 'PP', lateral_mpc=lateral_mpc)
    if ctrl == 'mpc':
        v_ = p['_mpc_vals']
        extra.update(pose_to_rear_axle_m=p['mpc_pose_to_rear_axle_m'],
                     mpc_cmd_max_lead_mps=p.get('mpc_cmd_max_lead_mps', .5),
                     mpc_speed_source=v_['mpc_speed_source'],
                     mpc_plan_lateral_accel_limit=v_['mpc_plan_lateral_accel_limit_mps2'],
                     mpc_fallback_max_ticks=max(1, round(v_['mpc_fallback_max_s'] * rate)),
                     mpc_latency_comp_s=v_['mpc_latency_comp_s'])
    c = Controller(**kw, **extra, logger_info=lambda *a, **k: None, logger_warn=lambda *a, **k: None)
    return c, p


def mpc_layers(cfg, state, path):
    """Speed limit layers of IntegratedMPC._reference evaluated at the car's projection:
    which one binds the first-stage target. Same formulas as the solver."""
    xy = path[:, :2] - state[:2]
    seg = np.diff(xy, axis=0)
    ds = np.linalg.norm(seg, axis=1)
    arc = np.concatenate(([0.], np.cumsum(ds)))
    frac = np.clip(-np.sum(xy[:-1] * seg, axis=1) / ds ** 2, 0., 1.)
    proj = xy[:-1] + frac[:, None] * seg
    idx = np.argmin(np.sum(proj ** 2, axis=1))
    start = arc[idx] + frac[idx] * ds[idx]
    from controller.mpc.integrated_mpc import _gradient
    tan = _gradient(xy, arc)
    yaw = np.unwrap(np.arctan2(tan[:, 1], tan[:, 0]))
    kap = _gradient(yaw, arc)
    plan = path[:, 2]
    lat = np.sqrt(cfg.max_lateral_accel / np.maximum(abs(kap), 1e-6))
    hard = np.minimum(np.minimum(plan, cfg.max_speed), lat)
    soft = np.minimum(hard, np.sqrt(.85 * cfg.max_lateral_accel / np.maximum(abs(kap), 1e-6)))
    brk = 2 * cfg.braking_preview_ratio * cfg.max_decel * arc
    caps = np.sqrt(np.maximum(0., np.minimum.accumulate((soft ** 2 + brk)[::-1])[::-1] - brk))
    f = lambda a: float(np.interp(start, arc, a))
    return {'plan': f(plan), 'max_speed': cfg.max_speed, 'lat_accel': f(lat), 'lat_soft': f(soft),
            'brake_preview': f(caps), 'kappa_path': f(kap)}


def simulate(ctrl='pp', profile='5p0', overrides=None, rate=50, laps=2, t_max=60.0, log_every=0,
             servo_delay_ticks=1, noise_pos=0.0, noise_v=0.0, offset_d=0.0, offset_yaw=0.0, seed=0, pose_delay_ticks=0, noise_yaw=0.0, cpu_miss=0.0):
    rng = np.random.default_rng(seed)
    c, p = build(ctrl, profile, overrides or {}, rate)
    conv = c.converter
    dt = 1.0 / rate
    L = p.get('wheelbase', 0.324)
    x, y, yaw = FULL[0, 0], FULL[0, 1], FULL[0, 6]
    x -= offset_d * np.sin(yaw)
    y += offset_d * np.cos(yaw)
    yaw += offset_yaw
    v, delta = 0.0, 0.0
    delta_queue = collections.deque([0.0] * servo_delay_ticks)
    acc_now = np.zeros(10)
    s_prev = float(conv.get_frenet([x], [y])[0][0])
    s_total, t = 0.0, 0.0
    lap_marks = []
    rec = collections.defaultdict(list)
    stops, stop_prev = 0, False
    statuses = collections.Counter()
    binding = collections.Counter()
    hist = collections.deque()
    if ctrl == 'mpc' and HISTORY:
        from controller.mpc.pose_prediction import PosePrediction
        c.mpc_history_prediction = True
        c.pose_prediction = PosePrediction(clock=lambda: SIMT[0])
    while t < t_max:
        hist.append((x, y, yaw, v))
        while len(hist) > pose_delay_ticks + 1: hist.popleft()
        px, py, pyaw, pv = hist[0]
        s_arr, d_arr = conv.get_frenet([x], [y])
        s, d = float(s_arr[0]), float(d_arr[0])
        ds = (s - s_prev + TRACK_LEN / 2) % TRACK_LEN - TRACK_LEN / 2
        s_total += ds
        s_prev = s
        if s_total >= TRACK_LEN * (len(lap_marks) + 1):
            lap_marks.append(t)
            if len(lap_marks) >= laps:
                break
        wp = local_wpnts(s)
        bb = BIAS if 2.8 <= s <= 3.8 else 0.0
        px, py = px - bb*np.sin(pyaw), py + bb*np.cos(pyaw)
        xm, ym = px + noise_pos * rng.standard_normal(), py + noise_pos * rng.standard_normal()
        vm = v + noise_v * rng.standard_normal()
        pose = np.array([[xm, ym, pyaw + noise_yaw * rng.standard_normal()]])
        frenet = np.array([s, d, vm, 0.0])
        c.yaw_rate = v * np.tan(delta) / L
        out = c.main_loop('GB_TRACK', pose, wp, vm, None, frenet, acc_now, TRACK_LEN)
        spd, steer = float(out[0]), float(out[3])
        if ctrl == 'mpc':
            r = c.mpc_result
            statuses[r.status.split(':')[0] if not r.success else r.status] += 1
            rec['status'].append(r.status)
            rec['elapsed_ms'].append(1e3 * r.elapsed_s)
            if spd <= 0.0:
                c.reset_mpc_speed_command()       # controller_manager.mpc_output_guard
            c.record_applied_steering(steer)
            is_stop = not r.success
            if c.mpc_speed_source == 'jsnd':
                rec['pp_steer'].append(c.mpc_pp_steering)
                rec['lat_cap'].append(c.mpc_lat_cap_active)
                binding['lat_cap' if c.mpc_lat_cap_active else 'jsnd_plan'] += 1
            else:
                lay = mpc_layers(c.lateral_mpc.config, np.array([x, y, yaw, v]), wp)
            if c.mpc_speed_source == 'jsnd':
                pass
            elif r.success:
                names = ['plan', 'max_speed', 'lat_accel', 'lat_soft', 'brake_preview']
                vals = [lay[n] for n in names]
                m = min(vals)
                binding[names[int(np.argmin(vals))] if m < lay['plan'] - 1e-3 else 'plan'] += 1
            if c.mpc_speed_source != 'jsnd':
                for k_, v_ in lay.items():
                    rec['L_' + k_].append(v_)
        else:
            is_stop = spd <= 0.0
        if is_stop and not stop_prev:
            stops += 1
        stop_prev = is_stop
        # ---- vehicle ----
        delta_queue.append(float(np.clip(steer, -SERVO_LIMIT, SERVO_LIMIT)))
        cmd_delta = delta_queue.popleft()
        delta += float(np.clip(cmd_delta - delta, -SERVO_RATE * dt, SERVO_RATE * dt))
        a = float(np.clip((spd - v) / VESC_TAU, -VESC_DEC, VESC_ACC))
        v_new = max(0.0, v + a * dt)
        acc_now[1:] = acc_now[:-1]
        acc_now[0] = (v_new - v) / dt
        x += dt * v * np.cos(yaw)
        y += dt * v * np.sin(yaw)
        yaw += dt * v * np.tan(delta) / L
        v = v_new
        t += dt
        SIMT[0] = t
        rec['t'].append(t); rec['v'].append(v); rec['cmd'].append(spd); rec['steer'].append(steer)
        rec['delta'].append(delta); rec['d'].append(d); rec['s'].append(s)
        rec['plan_v'].append(float(wp[0, 2])); rec['stop'].append(is_stop)
        rec['yawrate'].append(v*np.tan(delta)/L); rec['status_ok'].append(0 if ctrl!='mpc' else int(c.mpc_result.success)); rec['ay'].append(v * v * abs(np.tan(delta)) / L)
        if log_every and len(rec['t']) % log_every == 0:
            print(f"t={t:5.2f} s={s:5.2f} v={v:4.2f} cmd={spd:4.2f} plan={wp[0,2]:4.2f} d={d:+.3f} steer={steer:+.3f}")
    R = {k: np.asarray(v_) for k, v_ in rec.items() if k != 'status'}
    R_status = rec.get('status', [])
    res = {'ctrl': ctrl, 'laps_done': len(lap_marks), 'lap_times': [round(lap_marks[0], 2) for _ in lap_marks[:1]] +
           [round(b - a_, 2) for a_, b in zip(lap_marks, lap_marks[1:])],
           'avg_speed': round(float(R['v'].mean()), 3), 'max_speed': round(float(R['v'].max()), 3),
           'lat_err_mean': round(float(abs(R['d']).mean()), 3), 'lat_err_max': round(float(abs(R['d']).max()), 3),
           'stops': stops, 'stop_ticks': int(R['stop'].sum()), 'ay_max': round(float(R['ay'].max()), 2),
           'sim_time': round(t, 2)}
    if len(lap_marks) >= 2:
        m = R['t'] > lap_marks[0]
        res['flying_lap_max_speed'] = round(float(R['v'][m].max()), 3)
        res['flying_lap_avg_speed'] = round(float(R['v'][m].mean()), 3)
        res['flying_lap_lat_err_max'] = round(float(abs(R['d'][m]).max()), 3)
    if ctrl == 'mpc':
        res['status'] = dict(statuses.most_common())
        res['speed_source'] = c.mpc_speed_source
        el = R['elapsed_ms']
        res['mpc_ms_mean_p99_max'] = [round(float(el.mean()), 2), round(float(np.percentile(el, 99)), 2), round(float(el.max()), 2)]
        tot = sum(binding.values()) or 1
        res['binding_layer_pct'] = {k: round(100 * n / tot, 1) for k, n in binding.most_common()}
    res['_status_seq'] = R_status
    return res, R


def main():
    global VESC_TAU, REAL_DEADLINES
    ap = argparse.ArgumentParser()
    ap.add_argument('--ctrl', default='both')
    ap.add_argument('--profile', default='5p0')
    ap.add_argument('--rate', type=int, default=50)
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--set', nargs='*', default=[])
    ap.add_argument('--log', type=int, default=0)
    ap.add_argument('--save', default='')
    ap.add_argument('--transitions', action='store_true')
    ap.add_argument('--real-deadlines', action='store_true', help='race.launch 0.8/0.4 x period MPC budgets')
    ap.add_argument('--servo-delay', type=int, default=1)
    ap.add_argument('--tau', type=float, default=0.12)
    ap.add_argument('--noise-pos', type=float, default=0.0)
    ap.add_argument('--noise-v', type=float, default=0.0)
    ap.add_argument('--offset-d', type=float, default=0.0)
    ap.add_argument('--offset-yaw', type=float, default=0.0)
    a = ap.parse_args()
    REAL_DEADLINES = a.real_deadlines
    ov = {}
    for kv in a.set:
        k, v_ = kv.split('=')
        ov[k] = yaml.safe_load(v_)
    out = {}
    for ctrl in (['pp', 'mpc'] if a.ctrl == 'both' else [a.ctrl]):
        VESC_TAU = a.tau
        res, R = simulate(ctrl, a.profile, ov, a.rate, a.laps, log_every=a.log, servo_delay_ticks=a.servo_delay,
                          noise_pos=a.noise_pos, noise_v=a.noise_v, offset_d=a.offset_d, offset_yaw=a.offset_yaw)
        seq = res.pop('_status_seq')
        if a.transitions:
            prev = None
            for i, st in enumerate(seq):
                if st != prev:
                    print(f'  t={(i+1)/a.rate:6.2f} {st}')
                prev = st
        print(json.dumps(res, ensure_ascii=False))
        out[ctrl] = R
    if a.save:
        np.savez(a.save, **{f'{c}_{k}': v_ for c, R in out.items() for k, v_ in R.items()})


if __name__ == '__main__':
    main()
