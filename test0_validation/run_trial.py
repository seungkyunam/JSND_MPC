#!/usr/bin/env python3
"""Closed-loop test0 validation with the production controller and static planner.

The obstacle input is injected at the tracked-obstacle message boundary. Detection,
clustering and the live ROS state-machine graph are outside this offline simulation;
the production Frenet spliner and production velocity-profile algorithm are executed.
"""
import argparse
import collections
import configparser
import csv
import dataclasses
import json
import math
import os
import sys
import types
from pathlib import Path

import numpy as np
import trajectory_planning_helpers as tph
import yaml
from PIL import Image
from scipy.ndimage import binary_erosion

ROOT = Path(__file__).resolve().parents[1]
MAP_NAME = os.environ.get('OFFLINE_MAP_NAME', 'test0')
MAP_DIR = (ROOT/'stack_master/maps'/MAP_NAME if (ROOT/'stack_master/maps'/MAP_NAME).exists()
           else ROOT/'jsnd_int/racing_stack/stack_master/maps'/MAP_NAME)
sys.path[:0] = [str(ROOT/'jsnd_int/racing_stack/controller'), str(ROOT/'controller/test'),
                str(ROOT/'race_utils/f110_utils/libs/frenet_conversion'),
                str(ROOT/'planner/spliner'), str(ROOT/'planner/grid_filter'),
                str(ROOT/'jsnd_int/tests_offline')]
# Some development shells also expose another checkout containing a regular
# `controller` package. A regular package wins over this repository's namespace
# package even when our path is first, silently mixing the old Controller with
# the current MPC. Pin the offline validation to this checkout explicitly.
controller_pkg = types.ModuleType('controller')
controller_pkg.__path__ = [str(ROOT/'jsnd_int/racing_stack/controller/controller')]
sys.modules['controller'] = controller_pkg
import controller.mpc
from offline_support import install_marker_shims
install_marker_shims()
os.environ['JSND_LAP_STACK_ROOT'] = str(ROOT/'jsnd_int/racing_stack/stack_master')
os.environ['JSND_LAP_MAP_ROOT'] = str(MAP_DIR)
os.environ['JSND_LAP_BASE_CONFIG'] = str(ROOT/'stack_master/config')
import lap_sim as sim
from f110_msgs.msg import Obstacle, OTWpntArray, Wpnt, WpntArray
from spliner.static_avoidance_node import StaticObstacleSpliner
from vel_planner.vel_planner import calc_vel_profile

MAP_META = yaml.safe_load((MAP_DIR/f'{MAP_NAME}.yaml').read_text())
MAP_IMAGE = np.asarray(Image.open(MAP_DIR/MAP_META['image']).convert('L'))
RAW = json.loads((MAP_DIR/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
TRACK_LEN = float(RAW[-1]['s_m'])
VEHICLE_LENGTH = .52
VEHICLE_WIDTH = .29
OBSTACLE_SIZE = .30
OBSTACLES = ((
    dict(id=101, label='corner_entry', s=15.18, d=0.00, size=OBSTACLE_SIZE),
    dict(id=102, label='straight', s=28.97, d=0.00, size=OBSTACLE_SIZE),
) if MAP_NAME == 'test0' else (
    # 11_18_6 has no true straight and is only 13.91 m long. Offset the two
    # obstacles by 0.10 m toward the narrow side so the unchanged 0.35 m
    # evasion distance fits inside the measured track bounds.
    dict(id=301, label='corner_a', s=2.09, d=-0.10, size=OBSTACLE_SIZE),
    dict(id=302, label='corner_b', s=7.25, d=0.10, size=OBSTACLE_SIZE),
))
OBSTACLE_LAYOUTS = {
    'baseline': OBSTACLES,
    # Independent feasible placement: a mild corner and the long straight.
    # Both retain >0.70 m on at least one side of the centre line after the
    # 0.30 m obstacle is placed.
    'alternate': (
        dict(id=201, label='mild_corner', s=9.00, d=0.00, size=OBSTACLE_SIZE),
        dict(id=202, label='long_straight', s=34.00, d=0.00, size=OBSTACLE_SIZE),
    ) if MAP_NAME == 'test0' else OBSTACLES,
}

_INI = configparser.ConfigParser()
_INI.read(ROOT/'stack_master/config/CAR/racecar_f110.ini')
_VEH = json.loads(_INI.get('GENERAL_OPTIONS', 'veh_params'))
_VEL_OPTS = json.loads(_INI.get('GENERAL_OPTIONS', 'vel_calc_opts'))
_DYN = ROOT/'stack_master/config/CAR/veh_dyn_info'
_GGV, _AX = tph.import_veh_dyn_info.import_veh_dyn_info(str(_DYN/'ggv.csv'), str(_DYN/'ax_max_machines.csv'))
_, _BRAKE = tph.import_veh_dyn_info.import_veh_dyn_info(str(_DYN/'ggv.csv'), str(_DYN/'b_ax_max_machines.csv'))


class Clock:
    def __init__(self): self.t = 0.0
    def now(self):
        outer = self
        class Now:
            @property
            def nanoseconds(self): return int(outer.t*1e9)
            def to_msg(self):
                from builtin_interfaces.msg import Time
                return Time(sec=int(outer.t), nanosec=int((outer.t%1)*1e9))
        return Now()


class Logger:
    def noop(self, *args, **kwargs): pass
    info = warn = warning = error = debug = noop


class MapFilter:
    """Same 4x4 free-space erosion as the configured secondary map check."""
    def __init__(self):
        self.eroded_image = binary_erosion(MAP_IMAGE > 250, structure=np.ones((4, 4)))
    def set_erosion_kernel_size(self, size): pass
    def is_point_inside(self, x, y):
        ix = int(np.floor((x-MAP_META['origin'][0])/MAP_META['resolution']))
        iy = MAP_IMAGE.shape[0]-1-int(np.floor((y-MAP_META['origin'][1])/MAP_META['resolution']))
        return bool(0 <= ix < MAP_IMAGE.shape[1] and 0 <= iy < MAP_IMAGE.shape[0]
                    and self.eroded_image[iy, ix])


def wpnt_array():
    out = WpntArray()
    for i, p in enumerate(RAW):
        out.wpnts.append(Wpnt(id=i, s_m=float(p['s_m']), d_m=float(p['d_m']),
            x_m=float(p['x_m']), y_m=float(p['y_m']), d_left=float(p['d_left']),
            d_right=float(p['d_right']), psi_rad=float(p['psi_rad']),
            kappa_radpm=float(p['kappa_radpm']), vx_mps=float(sim.FULL[i, 2]),
            ax_mps2=float(p['ax_mps2'])))
    return out


def build_planner():
    cfg = yaml.safe_load((ROOT/'stack_master/config/static_avoidance_planner.yaml').read_text())['static_avoidance_planner']['ros__parameters']
    n = StaticObstacleSpliner.__new__(StaticObstacleSpliner)
    n.name, n._clock, n._logger = 'static_avoidance_planner', Clock(), Logger()
    n.get_clock, n.get_logger = lambda: n._clock, lambda: n._logger
    n.map_filter, n._dbg_fh, n._dbg_last_log_sec = MapFilter(), None, 0.0
    for key, attr in n._PARAM_ATTRS.items():
        setattr(n, attr, cfg[key])
    n.resolution = cfg['spline_resolution']
    n.obstacles, n.hint_obs = [], None
    track = wpnt_array()
    n.gb_wpnts = n.gb_scaled_wpnts = track
    n.max_s, n.wpnt_dist = TRACK_LEN, float(np.median(np.diff(sim.FULL[:, 4])))
    for attr, field in [('gb_s','s_m'), ('gb_d_left','d_left'), ('gb_d_right','d_right'),
                        ('gb_vx','vx_mps'), ('gb_ax','ax_mps2'), ('gb_psi','psi_rad'),
                        ('gb_kappa','kappa_radpm'), ('gb_x','x_m'), ('gb_y','y_m')]:
        setattr(n, attr, np.asarray([getattr(w, field) for w in track.wpnts], float))
    n.gb_vmax = float(np.max(n.gb_vx))
    n.converter = sim.make_converter()
    n.cur_s = n.cur_d = n.cur_x = n.cur_y = 0.0
    n.cur_vs = n.cur_yaw = 0.0
    n.last_side = n.last_side_obs_id = None
    n.last_good_path = n.last_good_obs_id = None
    n.last_good_generated = 0
    n.latched_group = n.latched_move_start = None
    n.last_good_origin_s = n.last_good_reach = 0.0
    n.debug_log_enabled = False
    return n


def make_obstacle(spec, rng=None, noisy=False):
    o = Obstacle()
    ds = float(rng.normal(0, .015)) if noisy else 0.0
    dd = float(rng.normal(0, .015)) if noisy else 0.0
    o.id, o.is_static, o.size = spec['id'], True, spec['size']
    o.s_center, o.d_center = (spec['s']+ds) % TRACK_LEN, spec['d']+dd
    o.s_start, o.s_end = o.s_center-o.size/2, o.s_center+o.size/2
    o.d_left, o.d_right = o.d_center+o.size/2, o.d_center-o.size/2
    return o


def reprofile(path, current_speed):
    """Production state-machine update_velocity algorithm and CAR dynamics data."""
    if len(path.wpnts) < 3: return
    w = path.wpnts
    lengths = np.linalg.norm(np.diff(np.asarray([[p.x_m,p.y_m] for p in w]), axis=0), axis=1)
    if np.any(lengths <= 1e-6): return
    end_idx = int(np.searchsorted(sim.FULL[:,4], w[-1].s_m) % len(sim.FULL))
    vx = calc_vel_profile(ax_max_machines=_AX, kappa=np.asarray([p.kappa_radpm for p in w]),
        el_lengths=lengths, closed=False, drag_coeff=_VEH['dragcoeff'], m_veh=_VEH['mass'],
        b_ax_max_machines=_BRAKE, ggv=_GGV, v_max=_VEH['v_max'],
        filt_window=_VEL_OPTS['vel_profile_conv_filt_window'], dyn_model_exp=_VEL_OPTS['dyn_model_exp'],
        v_start=max(.1, current_speed), v_end=float(sim.FULL[end_idx,2]))
    vx = np.minimum(vx*1.2, _VEH['v_max'])
    for p, value in zip(w, vx): p.vx_mps = float(value)


def profile(name):
    base = {'mpc_speed_source':'integrated', 'mpc_history_prediction':True}
    if name == 'legacy057':
        return {**base, 'mpc_horizon':12, 'mpc_preview_dt':.05, 'mpc_time_scaled_objective':False}
    if name == 'final':
        return {**base, 'mpc_horizon':10, 'mpc_preview_dt':.98/9,
            'mpc_time_scaled_objective':True, 'mpc_q_speed':10.0, 'mpc_r_accel':.05,
            'mpc_cmd_accel_lead_mps':1.2, 'mpc_brake_lead_s':.08, 'mpc_steer_preview_s':.03}
    if name in ('n10','n15','n20','n10u','n15u','n20u'):
        n=int(name[1:3].rstrip('u'))
        return {**base, 'mpc_horizon':n, 'mpc_preview_dt':.98/(n-1),
            'mpc_preview_grid_power':1.5 if name.endswith('u') else 1.0,
            'mpc_time_scaled_objective':True, 'mpc_q_speed':10.0, 'mpc_r_accel':.05,
            'mpc_cmd_accel_lead_mps':1.2, 'mpc_brake_lead_s':.08, 'mpc_steer_preview_s':.03}
    if name == 'improved1s':
        return {**base, 'mpc_horizon':10, 'mpc_preview_dt':.98/9,
            'mpc_preview_grid_power':1.5, 'mpc_time_scaled_objective':True,
            'mpc_q_position':20.0, 'mpc_q_speed':20.0, 'mpc_r_accel':.05, 'mpc_cmd_accel_lead_mps':1.2,
            'mpc_brake_lead_s':.08, 'mpc_steer_preview_s':.03,
            'mpc_braking_recovery_max_mps':1.0, 'mpc_lateral_plan_fraction':.85,
            }
    if name == 'pp': return {}
    raise ValueError(name)


def scenario(name):
    if name == 'clear': return dict(obstacles=False, pose_delay=0, servo_delay=0, noise_xy=0., noise_yaw=0., noise_v=0., obstacle_noise=False, seed=0)
    if name == 'obstacles': return dict(obstacles=True, pose_delay=0, servo_delay=0, noise_xy=0., noise_yaw=0., noise_v=0., obstacle_noise=False, seed=0)
    if name == 'obstacles_realistic':
        return dict(obstacles=True, pose_delay=6, servo_delay=3, noise_xy=.02, noise_yaw=.01, noise_v=.05, obstacle_noise=True, seed=20261127)
    raise ValueError(name)


def signed_gap(a, b): return (a-b+TRACK_LEN/2)%TRACK_LEN-TRACK_LEN/2


def pp_physical_governor(wp):
    """Keep PP's waypoint-speed policy while bounding it by the shared tire limit."""
    out = wp.copy()
    k = np.abs(out[:,5])
    cap = np.sqrt(4.3/np.maximum(k, 1e-4))
    speed = np.minimum(out[:,2], cap)
    ds = np.maximum(np.linalg.norm(np.diff(out[:,:2], axis=0), axis=1), 1e-3)
    for i in range(len(speed)-2, -1, -1):
        speed[i] = min(speed[i], math.sqrt(speed[i+1]**2 + 2*5.0*ds[i]))
    out[:,2] = speed
    return out


def path_to_array(path, x, y, s, n=80):
    if path is None or not path.wpnts: return sim.local_wpnts(s, n)
    pts = path.wpnts
    dist = np.asarray([(p.x_m-x)**2+(p.y_m-y)**2 for p in pts])
    start = int(np.argmin(dist))
    chosen = list(pts[start:start+n])
    if len(chosen) < n:
        last_s = chosen[-1].s_m if chosen else s
        idx = int(np.searchsorted(sim.GB[:,4], last_s))
        chosen += [None]*(n-len(chosen))
        arr = []
        real_n = len(pts[start:start+n])
        for p in chosen[:real_n]: arr.append([p.x_m,p.y_m,p.vx_mps,0,p.s_m,p.kappa_radpm,p.psi_rad,p.ax_mps2,p.d_m])
        arr.extend(sim.GB[(np.arange(n-real_n)+idx+1)%len(sim.GB)].tolist())
        return np.asarray(arr, float)
    return np.asarray([[p.x_m,p.y_m,p.vx_mps,0,p.s_m,p.kappa_radpm,p.psi_rad,p.ax_mps2,p.d_m] for p in chosen], float)


def body_clearance(x, y, yaw, spec):
    idx = int(np.searchsorted(sim.FULL[:,4], spec['s'])) % len(sim.FULL)
    ox = sim.FULL[idx,0] - spec['d']*math.sin(sim.FULL[idx,6])
    oy = sim.FULL[idx,1] + spec['d']*math.cos(sim.FULL[idx,6])
    dx, dy = ox-x, oy-y
    bx = abs(math.cos(yaw)*dx + math.sin(yaw)*dy)
    by = abs(-math.sin(yaw)*dx + math.cos(yaw)*dy)
    qx, qy = max(bx-VEHICLE_LENGTH/2, 0), max(by-VEHICLE_WIDTH/2, 0)
    outside = math.hypot(qx,qy)
    inside = min(max(bx-VEHICLE_LENGTH/2, by-VEHICLE_WIDTH/2), 0)
    return outside+inside-spec['size']/2


def run(controller_name, profile_name, scenario_name, output, laps=11, path_refresh_s=None,
        extra_overrides=None, seed=None, obstacle_layout='baseline', local_waypoints=80,
        plant_max_speed_mps=23250/4614, control_rate_hz=50, prediction_s=None):
    kind = 'pp' if controller_name == 'pp' else 'mpc'
    overrides=profile(profile_name); overrides.update(extra_overrides or {})
    control_rate_hz = int(control_rate_hz)
    if control_rate_hz <= 0:
        raise ValueError('control_rate_hz must be positive')
    if kind == 'mpc' and prediction_s is not None:
        horizon = int(overrides.get('mpc_horizon', 10))
        preview_s = float(prediction_s)-1.0/control_rate_hz
        if horizon < 2 or preview_s <= 0.:
            raise ValueError('prediction_s must exceed one control period')
        overrides['mpc_preview_dt'] = preview_s/(horizon-1)
    c, params = sim.build(kind, '5p0', overrides, control_rate_hz)
    if kind == 'mpc':
        c.mpc_cmd_accel_lead_mps = params['mpc_cmd_accel_lead_mps']
        c.lateral_mpc.set_global_path(sim.GB)
    cfg, planner = scenario(scenario_name), build_planner()
    if seed is not None: cfg['seed'] = int(seed)
    rng = np.random.default_rng(cfg['seed'])
    obstacles = OBSTACLE_LAYOUTS[obstacle_layout]
    if path_refresh_s is not None: planner.path_refresh_s=float(path_refresh_s)
    planner.last_good_geometry_time=-np.inf
    dt, wheelbase = 1.0/control_rate_hz, .324
    x,y,yaw = map(float, sim.FULL[0,[0,1,6]])
    v = delta = t = total = 0.0
    prev_s = float(c.converter.get_frenet([x],[y])[0,0])
    poses = collections.deque([(x,y,yaw)]*max(1,cfg['pose_delay']))
    servo = collections.deque([0.]*max(1,cfg['servo_delay']))
    obstacle_measurements = collections.deque([[make_obstacle(o) for o in obstacles]]*max(1,cfg['pose_delay']))
    accel_history, lap_marks, rows, planner_log = np.zeros(10), [], [], []
    active_path = None
    c.pose_prediction.clock = lambda: t
    terminal_stop = False
    for tick in range(15000):
        s,d = map(float, c.converter.get_frenet([x],[y])[:,0])
        total += signed_gap(s, prev_s); prev_s = s
        while total >= TRACK_LEN*(len(lap_marks)+1): lap_marks.append(t)
        if len(lap_marks) >= laps: break
        if cfg['obstacles'] and tick % max(1, round(.1*control_rate_hz)) == 0:
            measured = [make_obstacle(o,rng,cfg['obstacle_noise']) for o in obstacles]
            if cfg['pose_delay']:
                obstacle_measurements.append(measured); measured = obstacle_measurements.popleft()
            planner._clock.t, planner.cur_s, planner.cur_d = t, s, d
            planner.cur_x, planner.cur_y, planner.cur_yaw, planner.cur_vs = x,y,yaw,v
            planner.obstacles = measured
            dbg = {}
            planned = planner._select_stable_path(planner._plan(dbg), dbg)
            if planned.wpnts:
                planner.last_good_path=planned; planner.last_good_obs_id=dbg.get('obs_id')
                if not dbg.get('stable_reuse',False):
                    planner.last_good_generated=int(dbg.get('generated_count',len(planned.wpnts)))
                    planner.last_good_origin_s=s; planner.last_good_reach=float(dbg.get('reach',0.0))
                reprofile(planned, v); active_path = planned
            else: active_path = None
            safe_dbg = {}
            for key, value in dbg.items():
                if isinstance(value, (str, bool, int, float, np.integer, np.floating)) or value is None:
                    safe_dbg[key] = value.item() if isinstance(value, (np.integer, np.floating)) else value
                elif key in ('obs', 'target'):
                    safe_dbg[key] = int(value.id)
            planner_log.append(dict(t=t, lap=len(lap_marks)+1, **safe_dbg))
        wp = path_to_array(active_path,x,y,s,n=local_waypoints)
        if kind == 'pp': wp = pp_physical_governor(wp)
        if cfg['pose_delay']:
            poses.append((x,y,yaw)); px,py,pyaw=poses.popleft()
        else: px,py,pyaw=x,y,yaw
        measured_v=max(0.,v+cfg['noise_v']*rng.standard_normal())
        pose=np.array([[px+cfg['noise_xy']*rng.standard_normal(),py+cfg['noise_xy']*rng.standard_normal(),pyaw+cfg['noise_yaw']*rng.standard_normal()]])
        c.yaw_rate=v*np.tan(delta)/wheelbase; c.pose_prediction.observe(t,c.yaw_rate)
        out=c.main_loop('OVERTAKE' if active_path is not None else 'GB_TRACK',pose,wp,measured_v,None,np.array([s,d,measured_v,0.]),accel_history,TRACK_LEN)
        command,steer=float(out[0]),float(out[3])
        status,solver_ms,elapsed_ms,iterations,failure='PP',0.,0.,0,False
        optimal_accel = applied_accel = predicted_first = predicted_peak = float('nan')
        lead_mps = command_upper = float('nan')
        if kind=='mpc':
            res=c.mpc_result; status,solver_ms,elapsed_ms,iterations=res.status,res.solver_s*1000,res.elapsed_s*1000,res.iterations
            failure=not res.success
            diag=c.mpc_speed_diagnostics
            optimal_accel=float(diag.get('optimal_accel_mps2',float('nan')))
            applied_accel=float(diag.get('applied_accel_mps2',float('nan')))
            predicted_first=float(diag.get('predicted_first_mps',float('nan')))
            predicted_peak=float(diag.get('predicted_peak_mps',float('nan')))
            lead_mps=float(diag.get('lead_mps',float('nan')))
            command_upper=float(diag.get('command_upper_mps',float('nan')))
            if command<=0: c.reset_mpc_speed_command()
            c.record_applied_steering(steer)
        motor=float(np.clip(command,0.,plant_max_speed_mps))
        # Shared actuator/tire safety envelope. The old PP comparison allowed the
        # kinematic model to realize lateral accelerations that the tires cannot.
        lateral_steer_limit = math.atan(4.5*wheelbase/max(v*v, 1e-6))
        target=float(np.clip(steer,-min(sim.SERVO_LIMIT,lateral_steer_limit),min(sim.SERVO_LIMIT,lateral_steer_limit)))
        if cfg['servo_delay']: servo.append(target); target=servo.popleft()
        delta += float(np.clip(target-delta,-sim.SERVO_RATE*dt,sim.SERVO_RATE*dt))
        delta = float(np.clip(delta, -lateral_steer_limit, lateral_steer_limit))
        ay=v*v*abs(np.tan(delta))/wheelbase
        desired=(motor-v)/sim.VESC_TAU; long_lim=sim.VESC_ACC if desired>=0 else sim.VESC_DEC
        available=long_lim*math.sqrt(max(0.,1-(ay/4.5)**2)); accel=float(np.clip(desired,-available,available))
        combined=(accel/long_lim)**2+(ay/4.5)**2
        clearances=[body_clearance(x,y,yaw,o) for o in obstacles]
        path_speed_cap = float(wp[0, 2])
        if kind == 'mpc':
            try:
                arc, caps = c.lateral_mpc._geometry_caps(wp, np.inf)
                seg = np.diff(wp[:, :2], axis=0)
                frac = np.clip(np.sum((np.array([x, y])-wp[:-1, :2])*seg, axis=1)
                               / np.maximum(np.sum(seg*seg, axis=1), 1e-9), 0., 1.)
                projection = wp[:-1, :2]+frac[:, None]*seg
                j = int(np.argmin(np.linalg.norm(projection-np.array([x, y]), axis=1)))
                path_speed_cap = float(np.interp(arc[j]+frac[j]*(arc[j+1]-arc[j]), arc, caps))
            except (ValueError, IndexError, TypeError):
                path_speed_cap = float('nan')
        global_kappa = float(np.interp(s, sim.FULL[:, 4], sim.FULL[:, 5]))
        rows.append(dict(t=t,lap=len(lap_marks)+1,s=s,d=d,x=x,y=y,yaw=yaw,speed=v,measured_speed=measured_v,
            command_speed=command,motor_command=motor,command_steer=steer,actual_steer=delta,accel=accel,lateral_accel=ay,
            combined_limit=combined,global_kappa=global_kappa,path_speed_cap=path_speed_cap,
            mpc_optimal_accel=optimal_accel,mpc_applied_accel=applied_accel,
            mpc_predicted_first_speed=predicted_first,mpc_predicted_peak_speed=predicted_peak,
            mpc_command_lead=lead_mps,mpc_command_upper=command_upper,
            status=status,solver_ms=solver_ms,elapsed_ms=elapsed_ms,iterations=iterations,
            solver_failure=int(failure),avoidance_active=int(active_path is not None),clearance_corner_m=clearances[0],clearance_straight_m=clearances[1]))
        accel_history[1:]=accel_history[:-1]; accel_history[0]=accel
        x+=dt*v*math.cos(yaw); y+=dt*v*math.sin(yaw); yaw+=dt*v*math.tan(delta)/wheelbase
        v=max(0.,v+accel*dt); t=(tick+1)*dt
        stop_window = max(1, round(5.0*control_rate_hz))
        if t > 5.0 and len(rows) >= stop_window and max(r['speed'] for r in rows[-stop_window:]) < .1:
            terminal_stop = True
            break
    output.mkdir(parents=True,exist_ok=False)
    with (output/'trace.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
    with (output/'planner.jsonl').open('w') as f:
        for z in planner_log: f.write(json.dumps(z)+'\n')
    active=np.asarray([r['lap']>1 for r in rows]); speeds=np.asarray([r['speed'] for r in rows])[active]
    lap_times=np.diff(lap_marks); errors=np.asarray([r['d'] for r in rows])[active]
    solve=np.asarray([r['solver_ms'] for r in rows])[active]; elapsed=np.asarray([r['elapsed_ms'] for r in rows])[active]
    failures=np.asarray([r['solver_failure'] for r in rows])[active]
    collision=(np.asarray([min(r['clearance_corner_m'],r['clearance_straight_m'])<0 for r in rows])&active
               if cfg['obstacles'] else np.zeros(len(rows), dtype=bool))
    stopped=(np.asarray([r['speed'] for r in rows])<.1)&active
    events=lambda q:int(np.sum(q & ~np.r_[False,q[:-1]]))
    passages=[]
    for lap in range(2,laps+1) if cfg['obstacles'] else ():
        rr=[r for r in rows if r['lap']==lap]
        for spec,field in zip(obstacles,('clearance_corner_m','clearance_straight_m')):
            near=[r for r in rr if abs(signed_gap(r['s'],spec['s']))<1.5]
            before=[r['speed'] for r in rr if -1.5<signed_gap(r['s'],spec['s'])<-.5]
            after=[r['speed'] for r in rr if .5<signed_gap(r['s'],spec['s'])<1.5]
            passages.append(dict(lap=lap,obstacle=spec['label'],min_clearance_m=min((r[field] for r in near),default=None),
                speed_before_mps=float(np.mean(before)) if before else None,speed_after_mps=float(np.mean(after)) if after else None,
                collision=any(r[field]<0 for r in near),success=bool(near) and not any(r[field]<0 for r in near)))
    summary=dict(controller=controller_name,profile=profile_name,scenario=scenario_name,map=MAP_NAME,completed_laps=len(lap_marks),
        excluded_laps=1,lap_times_s=lap_times.tolist(),best_lap_s=float(np.min(lap_times)) if len(lap_times) else None,
        worst_lap_s=float(np.max(lap_times)) if len(lap_times) else None,
        mean_lap_s=float(np.mean(lap_times)) if len(lap_times) else None,lap_std_s=float(np.std(lap_times,ddof=1)) if len(lap_times)>1 else None,
        max_speed_mps=float(np.max(speeds)) if len(speeds) else None,mean_speed_mps=float(np.mean(speeds)) if len(speeds) else None,
        tracking_rmse_m=float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
        solver_ms={q:(float(np.percentile(solve,p)) if len(solve) else None) for q,p in [('p50',50),('p95',95),('p99',99),('max',100)]},
        cycle_overrun_pct=float(100*np.mean(elapsed>1000/control_rate_hz)) if len(elapsed) else None,
        solver_failure_pct=float(100*np.mean(failures)) if len(failures) else None,collision_events=events(collision), terminal_stop=terminal_stop,
        stop_events=events(stopped),physical_violation_pct=(float(100*np.mean(np.asarray([r['combined_limit'] for r in rows])[active]>1+1e-6)) if np.any(active) else None),
        avoidance_success_pct=float(100*np.mean([p['success'] for p in passages])) if passages else None,passages=passages,
        config=dataclasses.asdict(c.lateral_mpc.config) if kind=='mpc' else dict(pp_physical_governor_lat_mps2=4.3,brake_mps2=5.0),
        assumptions=dict(vehicle_length_m=VEHICLE_LENGTH,vehicle_width_m=VEHICLE_WIDTH,obstacle_size_m=OBSTACLE_SIZE,
            obstacle_layout=obstacle_layout, obstacle_placements=obstacles,
            local_waypoints=int(local_waypoints), plant_max_speed_mps=float(plant_max_speed_mps),
            control_rate_hz=control_rate_hz,
            prediction_s=(float(np.sum(c.lateral_mpc.periods)) if kind=='mpc' else None),
            planner_input='synthetic tracked Obstacle messages',full_perception_and_ros_state_machine=False,**cfg))
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary)); return 0 if len(lap_marks)==laps else 1


def main():
    p=argparse.ArgumentParser(); p.add_argument('--controller',choices=['pp','mpc'],required=True)
    p.add_argument('--profile',choices=['pp','legacy057','final','n10','n15','n20','n10u','n15u','n20u','improved1s'],required=True)
    p.add_argument('--scenario',choices=['clear','obstacles','obstacles_realistic'],required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--laps',type=int,default=11)
    p.add_argument('--path-refresh-s',type=float,default=None)
    p.add_argument('--seed',type=int,default=None)
    p.add_argument('--obstacle-layout',choices=sorted(OBSTACLE_LAYOUTS),default='baseline')
    p.add_argument('--local-waypoints',type=int,default=80)
    p.add_argument('--plant-max-speed-mps',type=float,default=23250/4614)
    p.add_argument('--rate',type=int,choices=(25,50,75,100),default=50)
    p.add_argument('--prediction-s',type=float,default=None)
    p.add_argument('--set',nargs='*',default=[],metavar='KEY=VALUE')
    a=p.parse_args(); extra={}
    for item in a.set:
        key,value=item.split('=',1); extra[key]=yaml.safe_load(value)
    return run(a.controller,a.profile,a.scenario,a.output,a.laps,a.path_refresh_s,
               extra,a.seed,a.obstacle_layout,a.local_waypoints,a.plant_max_speed_mps,
               a.rate,a.prediction_s)
if __name__=='__main__': raise SystemExit(main())
