#!/usr/bin/env python3
"""PP vs Integrated LTV-MPC comparison from vehicle rosbags (+ lap_analyser text logs).

usage (source ROS + JSND install first, or use cmp_analyze.sh):
  python3 cmp_analyze.py --bags <bag_dir>... [--layouts layouts.yaml] [--lapfiles] --out <dir>

Only measured values are reported. Anything a bag does not contain is written as empty
in CSV and "미측정" in Markdown -- never estimated. Per-run conditions come from
<bag>.meta.json (cmp_rec.sh) > --set overrides > the matching ~/bench_logs launch log header.

Metric definitions (all inside autodrive = /simple_mux/active_host == 'autodrive'):
  lap        : /lap_data lap_time if recorded, else Frenet-s wrap (s jumps > 0.5 track length).
               A lap counts only if >= 99 % of its interval is autodrive.
  lat RMSE   : /car_state/odom_frenet d (to /global_waypoints raceline), samples with v > 0.3.
               Also GB_TRACK-only (/state_machine) to exclude intentional avoidance offsets.
  speed      : /car_state/odom twist.linear.x, mean over samples with v > 0.1, and max.
  steering   : /vesc/ackermann_cmd (mux output) steering_angle; rate = diff/dt (0 < dt < 0.1 s).
  saturation : |delta| >= own_limit - 0.005 (PP 0.53 rad clip, MPC 0.4 rad), and |delta| >= 0.395.
  accel/jerk : odom speed resampled at 50 Hz, 0.1 s moving average, then d/dt (and again).
  compute    : MPC /controller/mpc_elapsed_ms; PP /controller/latency (Hz -> ms) only if present.
  frequency  : rate of /vesc/high_level/ackermann_cmd (controller output) in autodrive.
  stop       : v < 0.1 for >= 0.5 s after the car first exceeded 0.3 m/s in that autodrive segment.
  clearance  : min valid /scan range (LiDAR origin, NOT car body) in autodrive.
  avoidance  : per layout obstacle s_o, each pass of [s_o-pre, s_o+post] in autodrive.
               success = no stop, no autodrive loss and min scan >= contact_m in the window.
"""
import argparse, bisect, csv, glob, json, math, os, re, sys
from datetime import datetime
import numpy as np

HOME = os.path.expanduser('~')
TOPICS = ['/car_state/odom', '/car_state/odom_frenet', '/vesc/ackermann_cmd',
          '/vesc/high_level/ackermann_cmd', '/simple_mux/active_host', '/simple_mux/estop_state',
          '/controller/mpc_status', '/controller/mpc_elapsed_ms', '/controller/mpc_loop_period_ms',
          '/controller/latency', '/scan', '/state_machine', '/global_waypoints', '/lap_data']
STEER_LIMIT = {'pp': 0.53, 'mpc': 0.4}
NA = '미측정'


# ----------------------------------------------------------------------------- reading
def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    meta = os.path.join(path, 'metadata.yaml')
    if not os.path.exists(meta):
        raise RuntimeError(f'{path}: metadata.yaml 없음 (녹화 중단). 사본에서 `ros2 bag reindex <copy> -s mcap` 후 사용')
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    avail = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in TOPICS if t in avail]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    types = {t: get_message(avail[t]) for t in want}
    d = {k: [] for k in ('odom', 'fren', 'out', 'ctl', 'host', 'estop', 'status', 'elapsed',
                         'period', 'latency', 'scan', 'sm', 'lap')}
    track_len = None
    while r.has_next():
        topic, raw, t = r.read_next()
        t *= 1e-9
        m = deserialize_message(raw, types[topic])
        if topic == '/car_state/odom':
            d['odom'].append((t, m.twist.twist.linear.x))
        elif topic == '/car_state/odom_frenet':
            d['fren'].append((t, m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == '/vesc/ackermann_cmd':
            d['out'].append((t, m.drive.speed, m.drive.steering_angle))
        elif topic == '/vesc/high_level/ackermann_cmd':
            d['ctl'].append((t, m.drive.speed, m.drive.steering_angle))
        elif topic == '/simple_mux/active_host':
            d['host'].append((t, m.data))
        elif topic == '/simple_mux/estop_state':
            d['estop'].append((t, m.data))
        elif topic == '/controller/mpc_status':
            d['status'].append((t, m.data))
        elif topic == '/controller/mpc_elapsed_ms':
            d['elapsed'].append((t, m.data))
        elif topic == '/controller/mpc_loop_period_ms':
            d['period'].append((t, m.data))
        elif topic == '/controller/latency':
            d['latency'].append((t, m.data))
        elif topic == '/scan':
            rg = np.asarray(m.ranges, dtype=float)
            ok = np.isfinite(rg) & (rg > max(m.range_min, 0.02)) & (rg < m.range_max)
            d['scan'].append((t, float(rg[ok].min()) if ok.any() else math.nan))
        elif topic == '/state_machine':
            d['sm'].append((t, m.data))
        elif topic == '/global_waypoints' and track_len is None and len(m.wpnts):
            track_len = float(m.wpnts[-1].s_m)
        elif topic == '/lap_data':
            d['lap'].append((t, m.lap_count, m.lap_time, m.average_lateral_error_to_global_waypoints,
                             m.max_lateral_error_to_global_waypoints))
    return d, track_len, sorted(avail)


# ----------------------------------------------------------------------------- run meta
LOG_PATTERNS = [
    (re.compile(r'step45_(pp|mpc|improved)_([A-Za-z0-9]+)_(\d{8}_\d{6})\.log$'), 'step45'),
    (re.compile(r'race_(mpc|pp)([0-9p]*)(_integrated)?_(\d{8}_\d{6})\.log$'), 'race'),
]


def infer_from_roslog(t0):
    """Evidence from ~/.ros/log/<start>/launch.log of the launch running at t0:
    mpc_car.yaml loaded -> mpc (profile from mpc_profile_*.yaml), otherwise pp;
    map from the maps/<name>/ params-file path; install = JSND (original) or JSND_INT."""
    best = None
    for dpath in glob.glob(os.path.join(HOME, '.ros', 'log', '2026-*')):
        m = re.match(r'(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})', os.path.basename(dpath))
        f = os.path.join(dpath, 'launch.log')
        if not m or not os.path.exists(f):
            continue
        start = datetime.strptime(m.group(1), '%Y-%m-%d-%H-%M-%S').timestamp()
        if start <= t0 + 5 and os.path.getmtime(f) >= t0 and (best is None or start > best[0]):
            best = (start, f)
    if not best:
        return {}
    txt = open(best[1], errors='replace').read()
    if 'controller_manager' not in txt:
        return {}
    maps = re.findall(r'/maps/([A-Za-z0-9_]+)/', txt)
    prof = re.findall(r'mpc_profile_([A-Za-z0-9]+)\.yaml', txt)
    # Process cmd lines (params files) appear only when processes die with a message, so
    # absence of mpc_car.yaml is NOT evidence of PP. PP needs the original JSND controller
    # (no MPC code) or an explicit cmd without mpc_car.yaml.
    mpc = 'mpc_car.yaml' in txt
    orig = '/home/jsnd/JSND/install/controller/lib/controller/controller_manager' in txt
    cmd_seen = 'lib/controller/controller_manager --ros-args' in txt
    ctrl = 'mpc' if mpc else ('pp' if (orig or cmd_seen) else '')
    tag = 'mpc_car.yaml 로드' if mpc else ('원본 JSND controller_manager' if orig else
                                         ('controller cmd에 mpc_car.yaml 없음' if cmd_seen else 'cmd 기록 없음'))
    return {'controller': ctrl, 'profile': prof[0] if (mpc and prof) else ('pp_default' if ctrl == 'pp' else ''),
            'map': maps[0] if maps else '',
            'basis': f"~/.ros/log/{os.path.basename(os.path.dirname(best[1]))} ({tag})"}


def infer_from_logs(t0):
    """Latest ~/bench_logs launch log that started before the bag and was written after it."""
    best = None
    for f in glob.glob(os.path.join(HOME, 'bench_logs', '*.log')):
        name = os.path.basename(f)
        for rx, kind in LOG_PATTERNS:
            m = rx.search(name)
            if not m:
                continue
            ts = m.group(3) if kind == 'step45' else m.group(4)
            start = datetime.strptime(ts, '%Y%m%d_%H%M%S').timestamp()
            if start < datetime(2026, 1, 1).timestamp():  # clock not yet synced (1970 name)
                continue
            if start <= t0 + 5 and os.path.getmtime(f) >= t0 and (best is None or start > best[0]):
                if kind == 'step45':
                    ctrl, prof = m.group(1), m.group(2)
                else:
                    ctrl, prof = m.group(1), (m.group(2) or ('lowspeed' if m.group(1) == 'mpc' else ''))
                header = open(f, errors='replace').readline().strip()
                best = (start, f, ctrl, prof, header)
        m = re.search(r'race_pp_(\d{6})\.log$', name)  # date-less PP log: date from mtime
        if m:
            day = datetime.fromtimestamp(os.path.getmtime(f)).strftime('%Y%m%d')
            start = datetime.strptime(day + m.group(1), '%Y%m%d%H%M%S').timestamp()
            if start <= t0 + 5 and os.path.getmtime(f) >= t0 and (best is None or start > best[0]):
                best = (start, f, 'pp', 'pp_default', '')
    if not best:
        return {}
    _, f, ctrl, prof, header = best
    txt = open(f, errors='replace').read()
    has_mpc = bool(re.search(r'Integrated MPC selected|IntegratedConfig|SteeringMPC|mpc_car\.yaml', txt))
    maps = re.findall(r'/maps/([A-Za-z0-9_]+)/', txt)
    if ctrl == 'pp' and has_mpc:
        return {'note': f'{os.path.basename(f)}: 이름은 pp인데 MPC 흔적 있음 → 분류 보류'}
    out = {'controller': 'mpc' if ctrl in ('mpc', 'improved') else ctrl, 'profile': prof,
           'stack_log': f, 'meta_source': 'bench_logs launch log'}
    if ctrl == 'improved':
        out['note'] = 'improved mode (heading_robust) - Step 46 사용 금지 모드'
    hm = dict(kv.split('=', 1) for kv in header.split() if '=' in kv)
    out['map'] = hm.get('map') or (maps[0] if maps else '')
    out['basis'] = f"bench_logs/{os.path.basename(f)}" + (' (MPC 로그 확인)' if has_mpc else '')
    if 'checkout' in hm:
        out['checkout'] = hm['checkout'][:7]
    return out


def run_meta(bag, t0, overrides):
    meta = {'controller': '', 'profile': '', 'cap_label': '', 'layout': 'unknown', 'map': '',
            'checkout': '', 'note': '', 'meta_source': 'none'}
    side = bag.rstrip('/') + '.meta.json'
    if os.path.exists(side):
        j = json.load(open(side))
        meta.update({k: v for k, v in j.items() if v not in (None, '')})
        meta['meta_source'] = 'cmp_rec.sh meta.json'
        hm = dict(kv.split('=', 1) for kv in j.get('stack_log_header', '').split() if '=' in kv)
        meta.setdefault('map', hm.get('map', ''))
        if not meta.get('map'):
            meta['map'] = hm.get('map', '')
        meta['profile'] = meta.get('profile') or hm.get('profile', '')
        meta['checkout'] = meta.get('checkout') or hm.get('checkout', '')[:7]
    else:
        inf = infer_from_logs(t0)
        meta.update({k: v for k, v in inf.items() if v})
        ros = infer_from_roslog(t0)
        if ros:  # launch.log params are the stronger evidence for controller/map/profile
            if meta.get('controller') and meta['controller'] != ros['controller']:
                meta['note'] = (meta.get('note', '') + f" bench_log={meta['controller']} vs ros_log={ros['controller']}").strip()
            meta.update({k: ros[k] for k in ('controller', 'profile', 'map') if ros.get(k)})
            meta['meta_source'] = ros['basis'] + (' + ' + meta['meta_source'] if inf else '')
    meta.update(overrides)
    if not meta.get('cap_label'):
        meta['cap_label'] = meta.get('profile', '') or 'unknown'
    return meta


# ----------------------------------------------------------------------------- helpers
def auto_mask_fn(host):
    ht = [h[0] for h in host]

    def is_auto(t):
        i = bisect.bisect_right(ht, t) - 1
        return i >= 0 and host[i][1] == 'autodrive'
    return is_auto


def auto_segments(host, t_end):
    segs, start = [], None
    for t, h in host:
        if h == 'autodrive' and start is None:
            start = t
        elif h != 'autodrive' and start is not None:
            segs.append((start, t))
            start = None
    if start is not None:
        segs.append((start, t_end))
    return segs


def in_segs(t, segs):
    return any(a <= t <= b for a, b in segs)


def coverage(a, b, segs):
    tot = sum(max(0., min(b, y) - max(a, x)) for x, y in segs)
    return tot / (b - a) if b > a else 0.


def pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if len(x) else math.nan


def rms(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x * x))) if len(x) else math.nan


def movavg(x, n):
    if len(x) < n:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode='same')


# ----------------------------------------------------------------------------- metrics
def analyse(bag, layouts, args, overrides):
    d, track_len, topics = read_bag(bag)
    if not d['odom'] or not d['host']:
        raise RuntimeError(f'{bag}: /car_state/odom 또는 /simple_mux/active_host 없음')
    t0, t1 = d['odom'][0][0], d['odom'][-1][0]
    meta = run_meta(bag, t0, overrides)
    ctrl = meta.get('controller', '')
    segs = auto_segments(d['host'], t1)
    is_auto = auto_mask_fn(d['host'])
    auto_time = sum(b - a for a, b in segs)
    R = dict(run=os.path.basename(bag.rstrip('/')), bag=bag, **{k: meta.get(k, '') for k in
             ('controller', 'profile', 'cap_label', 'layout', 'map', 'checkout', 'note', 'meta_source')})
    R['recorded_s'] = round(t1 - t0, 1)
    R['autodrive_s'] = round(auto_time, 1)
    if track_len is None and d['fren']:
        track_len = max(f[1] for f in d['fren'])
    R['track_length_m'] = round(track_len, 3) if track_len else ''

    odom = np.array(d['odom'])
    ot, ov = odom[:, 0], odom[:, 1]
    amask = np.array([is_auto(t) for t in ot])

    # ---- laps
    laps = []
    if d['lap']:
        for t, n, lt, avg_e, max_e in d['lap']:
            if lt > 0 and coverage(t - lt, t, segs) >= 0.99:
                laps.append(dict(t_end=t, lap_time=lt, source='/lap_data', lap_avg_lat=avg_e, lap_max_lat=max_e))
    elif d['fren'] and track_len:
        fr = np.array(d['fren'])
        cross = [fr[i, 0] for i in range(1, len(fr))
                 if fr[i - 1, 1] - fr[i, 1] > 0.5 * track_len]
        for a, b in zip(cross[:-1], cross[1:]):
            if coverage(a, b, segs) >= 0.99:
                laps.append(dict(t_end=b, lap_time=b - a, source='frenet s wrap', lap_avg_lat='', lap_max_lat=''))
    lt = np.array([l['lap_time'] for l in laps]) if laps else np.array([])
    R['laps'] = len(laps)
    R['lap_best_s'] = round(float(lt.min()), 3) if len(lt) else ''
    R['lap_mean_s'] = round(float(lt.mean()), 3) if len(lt) else ''
    R['lap_std_s'] = round(float(lt.std(ddof=1)), 3) if len(lt) > 1 else ''
    R['lap_median_s'] = round(float(np.median(lt)), 3) if len(lt) else ''
    R['lap_source'] = laps[0]['source'] if laps else ''

    # ---- lateral error
    if d['fren']:
        fr = np.array(d['fren'])
        v_at = np.interp(fr[:, 0], ot, ov)
        m = np.array([is_auto(t) for t in fr[:, 0]]) & (v_at > 0.3)
        R['lat_rmse_m'] = round(rms(fr[m, 2]), 4) if m.any() else ''
        R['lat_abs_max_m'] = round(float(np.abs(fr[m, 2]).max()), 4) if m.any() else ''
        if d['sm']:
            smt = [s[0] for s in d['sm']]

            def state(t):
                i = bisect.bisect_right(smt, t) - 1
                return d['sm'][i][1] if i >= 0 else ''
            g = m & np.array([state(t) == 'GB_TRACK' for t in fr[:, 0]])
            R['lat_rmse_gbtrack_m'] = round(rms(fr[g, 2]), 4) if g.any() else ''
            R['gbtrack_share'] = round(float(g.sum() / max(1, m.sum())), 3)
        else:
            R['lat_rmse_gbtrack_m'] = ''
            R['gbtrack_share'] = ''
    else:
        R.update(lat_rmse_m='', lat_abs_max_m='', lat_rmse_gbtrack_m='', gbtrack_share='')

    # ---- speed
    va = ov[amask]
    mv = va[va > 0.1]
    R['speed_mean_mps'] = round(float(mv.mean()), 3) if len(mv) else ''
    R['speed_max_mps'] = round(float(va.max()), 3) if len(va) else ''

    # ---- steering (mux output)
    if d['out']:
        out = np.array(d['out'])
        om = np.array([is_auto(t) for t in out[:, 0]])
        st, stt = out[om, 2], out[om, 0]
        R['steer_rms_rad'] = round(rms(st), 4) if len(st) else ''
        dt = np.diff(stt)
        ok = (dt > 0) & (dt < 0.1)
        rate = np.diff(st)[ok] / dt[ok] if ok.any() else np.array([])
        R['steer_rate_rms_radps'] = round(rms(rate), 3) if len(rate) else ''
        lim = STEER_LIMIT.get(ctrl)
        R['steer_limit_rad'] = lim if lim else ''
        R['steer_sat_own_pct'] = round(100 * float(np.mean(np.abs(st) >= lim - 0.005)), 2) if (lim and len(st)) else ''
        R['steer_sat_0p395_pct'] = round(100 * float(np.mean(np.abs(st) >= 0.395)), 2) if len(st) else ''
    else:
        R.update(steer_rms_rad='', steer_rate_rms_radps='', steer_limit_rad='', steer_sat_own_pct='', steer_sat_0p395_pct='')

    # ---- accel / jerk from measured speed, per autodrive segment
    acc_all, jerk_all = [], []
    for a, b in segs:
        if b - a < 1.0:
            continue
        tt = np.arange(a, b, 0.02)
        v = movavg(np.interp(tt, ot, ov), 5)
        acc = movavg(np.gradient(v, 0.02), 5)
        jerk = np.gradient(acc, 0.02)
        acc_all.append(acc[5:-5])
        jerk_all.append(jerk[5:-5])
    acc = np.concatenate(acc_all) if acc_all else np.array([])
    jerk = np.concatenate(jerk_all) if jerk_all else np.array([])
    R['accel_rms_mps2'] = round(rms(acc), 3) if len(acc) else ''
    R['accel_max_mps2'] = round(float(acc.max()), 3) if len(acc) else ''
    R['decel_max_mps2'] = round(float(-acc.min()), 3) if len(acc) else ''
    R['jerk_rms_mps3'] = round(rms(jerk), 2) if len(jerk) else ''
    R['jerk_p95_abs_mps3'] = round(pct(np.abs(jerk), 95), 2) if len(jerk) else ''

    # ---- compute time / frequency
    comp = []
    if d['elapsed']:
        comp = [v for t, v in d['elapsed'] if is_auto(t)]
        R['compute_source'] = '/controller/mpc_elapsed_ms'
    elif d['latency']:
        comp = [1000.0 / v for t, v in d['latency'] if is_auto(t) and v > 0]
        R['compute_source'] = '/controller/latency (1/Hz)'
    else:
        R['compute_source'] = ''
    for q, k in ((50, 'p50'), (95, 'p95'), (99, 'p99')):
        R[f'compute_{k}_ms'] = round(pct(comp, q), 3) if comp else ''
    R['compute_max_ms'] = round(float(np.max(comp)), 3) if comp else ''
    if d['ctl']:
        ct = np.array([t for t, *_ in d['ctl'] if is_auto(t)])
        R['ctrl_rate_hz'] = round(len(ct) / auto_time, 2) if auto_time > 0 and len(ct) else ''
        p = np.diff(ct) * 1000
        p = p[p < 200]  # drop gaps between autodrive segments
        R['ctrl_period_p99_ms'] = round(pct(p, 99), 2) if len(p) else ''
    else:
        R['ctrl_rate_hz'] = R['ctrl_period_p99_ms'] = ''
    if d['status']:
        s = [x for t, x in d['status'] if is_auto(t)]
        R['mpc_solved_pct'] = round(100 * sum(1 for x in s if x.startswith('solved')) / len(s), 2) if s else ''
    else:
        R['mpc_solved_pct'] = ''

    # ---- stops
    stops = []
    for a, b in segs:
        m = (ot >= a) & (ot <= b)
        tt, vv = ot[m], ov[m]
        started, s0 = False, None
        for t, v in zip(tt, vv):
            if v > 0.3:
                started = True
            if not started:
                continue
            if v < 0.1:
                s0 = t if s0 is None else s0
            else:
                if s0 is not None and t - s0 >= 0.5:
                    stops.append((s0, t))
                s0 = None
        if s0 is not None and tt[-1] - s0 >= 0.5:
            stops.append((s0, tt[-1]))
    R['stops'] = len(stops)
    R['stopped_total_s'] = round(sum(b - a for a, b in stops), 2)
    R['stops_per_lap'] = round(len(stops) / len(laps), 3) if laps else ''
    R['takeovers'] = sum(1 for i in range(1, len(d['host']))
                         if d['host'][i - 1][1] == 'autodrive' and d['host'][i][1] == 'humandrive')

    # ---- clearance
    sc = np.array(d['scan']) if d['scan'] else np.zeros((0, 2))
    if len(sc):
        sm = np.array([is_auto(t) for t in sc[:, 0]])
        vals = sc[sm, 1]
        R['scan_min_m'] = round(float(np.nanmin(vals)), 3) if np.isfinite(vals).any() else ''
        R['scan_p1_m'] = round(pct(vals, 1), 3) if np.isfinite(vals).any() else ''
    else:
        R['scan_min_m'] = R['scan_p1_m'] = ''

    # ---- obstacle avoidance
    lay = layouts.get(meta.get('layout'))
    R['avoid_attempts'] = R['avoid_success'] = R['avoid_success_pct'] = R['avoid_min_scan_m'] = ''
    passes = []
    if lay and d['fren']:
        fr = np.array(d['fren'])
        pre, post = float(lay.get('pre_m', 1.5)), float(lay.get('post_m', 1.0))
        contact = float(lay.get('contact_m', 0.0))
        for ob in lay['obstacles']:
            so = float(ob['s'])
            lo, hi = so - pre, so + post
            inwin = (fr[:, 1] >= lo) & (fr[:, 1] <= hi) if lo >= 0 else ((fr[:, 1] >= lo % track_len) | (fr[:, 1] <= hi))
            idx = np.flatnonzero(inwin)
            if not len(idx):
                continue
            groups = np.split(idx, np.flatnonzero(np.diff(idx) > 5) + 1)
            for g in groups:
                ta, tb = fr[g[0], 0], fr[g[-1], 0]
                if not is_auto(ta):
                    continue
                ok_auto = coverage(ta, tb, segs) >= 0.99
                stopped = any(s0 < tb and s1 > ta for s0, s1 in stops)
                w = sc[(sc[:, 0] >= ta) & (sc[:, 0] <= tb), 1] if len(sc) else np.array([])
                mn = float(np.nanmin(w)) if len(w) and np.isfinite(w).any() else math.nan
                success = ok_auto and not stopped and (not np.isfinite(mn) or mn >= contact)
                passes.append(dict(run=R['run'], obstacle=ob.get('id', so), s=so, t_start=round(ta, 2),
                                   t_end=round(tb, 2), autodrive_kept=ok_auto, stopped=stopped,
                                   min_scan_m=round(mn, 3) if np.isfinite(mn) else '', success=success))
        if passes:
            R['avoid_attempts'] = len(passes)
            R['avoid_success'] = sum(p['success'] for p in passes)
            R['avoid_success_pct'] = round(100 * R['avoid_success'] / len(passes), 1)
            m = [p['min_scan_m'] for p in passes if p['min_scan_m'] != '']
            R['avoid_min_scan_m'] = min(m) if m else ''

    # ---- per-sample series for plots
    series = dict(laps=lt.tolist(), steer=(np.array(d['out'])[:, 2][[is_auto(t) for t in np.array(d['out'])[:, 0]]].tolist()
                                         if d['out'] else []),
                  compute=list(comp), speed_s=None)
    if d['fren'] and track_len:
        fr = np.array(d['fren'])
        m = np.array([is_auto(t) for t in fr[:, 0]])
        v_at = np.interp(fr[:, 0], ot, ov)
        bins = np.linspace(0, track_len, 71)
        k = np.digitize(fr[m, 1], bins) - 1
        prof = [float(np.median(v_at[m][k == i])) if np.any(k == i) else math.nan for i in range(70)]
        series['speed_s'] = (((bins[:-1] + bins[1:]) / 2).tolist(), prof)
    lap_rows = [dict(run=R['run'], controller=ctrl, cap_label=R['cap_label'], layout=R['layout'],
                     lap=i + 1, lap_time_s=round(l['lap_time'], 3), source=l['source'],
                     lap_avg_lat_m=l['lap_avg_lat'], lap_max_lat_m=l['lap_max_lat']) for i, l in enumerate(laps)]
    R['topics_missing'] = ' '.join(t for t in TOPICS if t not in topics)
    return R, lap_rows, passes, series


# ----------------------------------------------------------------------------- lap_analyser text logs
def lapfiles(directory):
    rows = []
    for f in sorted(glob.glob(os.path.join(directory, 'lap_analyzer_*.txt'))):
        txt = open(f, errors='replace').read()
        m = re.search(r'Laps done on (\d+ \w+ \d+:\d+:\d+)', txt)
        if not m:
            continue
        start = datetime.strptime(f'2026 {m.group(1)}', '%Y %d %b %H:%M:%S').timestamp()
        laps = [float(x) for x in re.findall(r'Lap #\d+: ([0-9.]+)\s*\n', txt)]
        if not laps:
            continue
        valid = start >= datetime(2026, 1, 1).timestamp()
        inf = infer_from_logs(start) if valid else {}
        ros = infer_from_roslog(start) if valid else {}
        ctrl = prof = mp = ''
        basis = '매칭 launch 로그 없음(시계 미동기 포함)'
        if inf.get('controller'):
            ctrl, prof, mp, basis = inf['controller'], inf.get('profile', ''), inf.get('map', ''), inf['basis']
        elif ros.get('controller'):
            ctrl, prof, mp, basis = ros['controller'], ros['profile'], ros['map'], ros['basis']
        elif valid and start < datetime(2026, 11, 22).timestamp():
            # MPC was first wired into the driving stack on 2026-11-22 (JSND_INT 0406352).
            ctrl, prof, basis = 'pp', 'pp_default', 'MPC 통합(11-22) 이전 날짜 → 원본 JSND(PP)'
        if not mp and ros.get('map'):
            mp = ros['map']
        if inf.get('note'):
            basis += ' / ' + inf['note']
        avg = re.findall(r'Avg Lat Error over the past 10 laps: Mean: ([0-9.]+)', txt)
        for i, lt in enumerate(laps):
            rows.append(dict(file=os.path.basename(f), session_start=datetime.fromtimestamp(start).strftime('%m-%d %H:%M:%S'),
                             controller=ctrl, controller_basis=basis, profile=prof, map=mp, lap=i + 1,
                             lap_time_s=lt, session_avg_lat_10laps_m=avg[0] if avg else ''))
    return rows


# ----------------------------------------------------------------------------- output
def write_csv(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)


MD_ROWS = [('랩 수', 'laps'), ('최고 랩 [s]', 'lap_best_s'), ('평균 랩 [s]', 'lap_mean_s'),
           ('랩 표준편차 [s]', 'lap_std_s'), ('횡오차 RMSE [m]', 'lat_rmse_m'),
           ('횡오차 RMSE GB_TRACK [m]', 'lat_rmse_gbtrack_m'), ('평균 속도 [m/s]', 'speed_mean_mps'),
           ('최고 속도 [m/s]', 'speed_max_mps'), ('조향 RMS [rad]', 'steer_rms_rad'),
           ('조향 변화율 RMS [rad/s]', 'steer_rate_rms_radps'), ('가속도 RMS [m/s²]', 'accel_rms_mps2'),
           ('최대 가속 [m/s²]', 'accel_max_mps2'), ('최대 감속 [m/s²]', 'decel_max_mps2'),
           ('Jerk RMS [m/s³]', 'jerk_rms_mps3'), ('계산시간 p50 [ms]', 'compute_p50_ms'),
           ('계산시간 p99 [ms]', 'compute_p99_ms'), ('실제 제어 주파수 [Hz]', 'ctrl_rate_hz'),
           ('MPC solved [%]', 'mpc_solved_pct'), ('회피 성공률 [%]', 'avoid_success_pct'),
           ('회피 시도', 'avoid_attempts'), ('정지 횟수', 'stops'), ('정지/랩', 'stops_per_lap'),
           ('수동 개입', 'takeovers'), ('최소 LiDAR 거리 [m]', 'scan_min_m'),
           ('조향 포화(자기 한계) [%]', 'steer_sat_own_pct'), ('조향 포화(≥0.395) [%]', 'steer_sat_0p395_pct')]


def fmt(v):
    return NA if v in ('', None) or (isinstance(v, float) and math.isnan(v)) else str(v)


def write_md(path, runs, lap_rows_txt, conds):
    L = ['# PP vs Integrated LTV-MPC 비교 결과 (자동 생성)', '',
         f'생성: {datetime.now():%Y-%m-%d %H:%M}. 측정값만 표기, 없는 값은 "{NA}". 정의는 `cmp_analyze.py` 머리말.', '']
    L.append('## 비교 조건별 요약 (같은 cap_label·layout·map끼리만 비교)')
    for key, rs in conds.items():
        cap, layout, mp = key
        ctrls = sorted({r['controller'] for r in rs})
        fair = 'PP·MPC 모두 있음' if {'pp', 'mpc'} <= set(ctrls) else f'**비교 불가: {"/".join(ctrls) or "?"}만 있음**'
        L += ['', f'### cap={cap} · layout={layout} · map={mp or "?"} — {fair}', '']
        head = '| 항목 | ' + ' | '.join(f"{r['controller']} `{r['run']}`" for r in rs) + ' |'
        L += [head, '|' + '---|' * (len(rs) + 1)]
        for label, k in MD_ROWS:
            L.append(f'| {label} | ' + ' | '.join(fmt(r.get(k)) for r in rs) + ' |')
        L.append('| 조건 출처 | ' + ' | '.join(f"{r['meta_source']}, profile={r['profile'] or '?'}, {r['checkout'] or ''}" for r in rs) + ' |')
        miss = {r['run']: r['topics_missing'] for r in rs if r['topics_missing']}
        if miss:
            L.append('')
            L += [f'- `{k}` 미기록 토픽: {v}' for k, v in miss.items()]
    if lap_rows_txt:
        L += ['', '## lap_analyser 텍스트 기록 (bag 없음 — 랩타임만, 수동 개입 랩 구분 불가)', '',
              '주의: lap_analyser는 자율/수동 구분 없이 결승선 통과 간격을 기록한다. 정지·수동 개입 랩이 섞여 있어 '
              '평균·표준편차는 왜곡된다(최고·중앙값 위주로 볼 것). bag 기반 표와 같은 조건 비교가 아니다.', '']
        pooled = {}
        for r in lap_rows_txt:
            if r['map'] == '11_18_6' and r['controller']:
                pooled.setdefault((r['controller'], r['profile']), []).append(r['lap_time_s'])
        if pooled:
            L += ['### 11_18_6 맵 컨트롤러·프로파일별 합계', '',
                  '| 컨트롤러 | 프로파일 | 세션 수 | 랩 수 | 최고 [s] | 중앙값 [s] |', '|---|---|---|---|---|---|']
            for (c, p), v in sorted(pooled.items()):
                ns = len({r['file'] for r in lap_rows_txt if r['map'] == '11_18_6' and r['controller'] == c and r['profile'] == p})
                L.append(f'| {c} | {p} | {ns} | {len(v)} | {min(v):.3f} | {np.median(v):.3f} |')
            L.append('')
        L += ['### 세션별', '',
              '| 세션 | 컨트롤러 | 프로파일 | 맵 | 분류 근거 | 랩 수 | 최고 [s] | 중앙값 [s] | 평균 [s] | 표준편차 [s] |',
              '|---|---|---|---|---|---|---|---|---|---|']
        by = {}
        for r in lap_rows_txt:
            by.setdefault((r['file'], r['session_start'], r['controller'], r['profile'], r['map'], r['controller_basis']),
                          []).append(r['lap_time_s'])
        for (f, s, c, p, mp, b), v in by.items():
            v = np.array(v)
            L.append(f'| {s} `{f}` | {c or "?"} | {p} | {mp or "?"} | {b} | {len(v)} | {v.min():.3f} | {np.median(v):.3f} | '
                     f'{v.mean():.3f} | {v.std(ddof=1) if len(v) > 1 else math.nan:.3f} |')
    open(path, 'w').write('\n'.join(L) + '\n')


def plots(out, runs, series, conds):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    col = {'pp': '#1f77b4', 'mpc': '#d62728'}
    for key, rs in conds.items():
        tag = '_'.join(x or 'na' for x in key)
        names = [f"{r['controller']}\n{r['run'][-15:]}" for r in rs]
        colors = [col.get(r['controller'], 'gray') for r in rs]
        # lap times
        fig, ax = plt.subplots(figsize=(7, 4))
        data = [series[r['run']]['laps'] for r in rs]
        if any(len(x) for x in data):
            ax.boxplot([x if len(x) else [math.nan] for x in data], labels=names)
            for i, x in enumerate(data):
                ax.scatter(np.full(len(x), i + 1), x, color=colors[i], s=14, zorder=3)
        ax.set_ylabel('lap time [s]'); ax.set_title(f'Lap time ({tag})'); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(out, f'lap_times_{tag}.png'), dpi=150); plt.close(fig)
        # key metric bars (missing -> no bar, labelled n/a)
        keys = [('lat_rmse_m', 'lat RMSE [m]'), ('speed_mean_mps', 'mean v [m/s]'), ('steer_rms_rad', 'steer RMS [rad]'),
                ('steer_rate_rms_radps', 'steer rate RMS [rad/s]'), ('jerk_rms_mps3', 'jerk RMS [m/s³]'),
                ('compute_p99_ms', 'compute p99 [ms]'), ('ctrl_rate_hz', 'control rate [Hz]'),
                ('stops_per_lap', 'stops / lap'), ('steer_sat_own_pct', 'steer sat. [%]')]
        fig, axs = plt.subplots(3, 3, figsize=(11, 8))
        for ax, (k, lab) in zip(axs.ravel(), keys):
            vals = [r.get(k) for r in rs]
            for i, v in enumerate(vals):
                if v not in ('', None):
                    ax.bar(i, float(v), color=colors[i])
                else:
                    ax.text(i, 0, 'n/a', ha='center', va='bottom', fontsize=8)
            ax.set_xticks(range(len(rs))); ax.set_xticklabels([r['controller'] for r in rs], fontsize=8)
            ax.set_title(lab, fontsize=9); ax.grid(axis='y', alpha=.3)
        fig.suptitle(f'PP vs MPC ({tag})'); fig.tight_layout(); fig.savefig(os.path.join(out, f'metrics_{tag}.png'), dpi=150); plt.close(fig)
        # speed profile
        fig, ax = plt.subplots(figsize=(8, 3.5))
        for r, c in zip(rs, colors):
            sp = series[r['run']]['speed_s']
            if sp:
                ax.plot(sp[0], sp[1], color=c, label=f"{r['controller']} {r['run'][-15:]}")
        ax.set_xlabel('s [m]'); ax.set_ylabel('median v [m/s]'); ax.legend(fontsize=7); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(out, f'speed_profile_{tag}.png'), dpi=150); plt.close(fig)
        # steering + compute histograms
        fig, axs = plt.subplots(1, 2, figsize=(10, 3.5))
        for r, c in zip(rs, colors):
            s = series[r['run']]
            if s['steer']:
                axs[0].hist(s['steer'], bins=60, histtype='step', color=c, density=True, label=r['controller'])
            if s['compute']:
                axs[1].hist(s['compute'], bins=60, histtype='step', color=c, density=True, label=r['controller'])
        axs[0].set_xlabel('steering [rad]'); axs[1].set_xlabel('compute time [ms]')
        for ax in axs:
            ax.legend(fontsize=7); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(out, f'steer_compute_{tag}.png'), dpi=150); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', nargs='*', default=[])
    ap.add_argument('--layouts', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'layouts.yaml'))
    ap.add_argument('--set', nargs='*', default=[], help='override meta for ALL bags, e.g. layout=L1 cap_label=cap1p0')
    ap.add_argument('--lapfiles', nargs='?', const=os.path.join(
        HOME, 'JSND/src/racing_stack/race_utils/f110_utils/nodes/lap_analyser/data'), default=None)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    layouts = {}
    if os.path.exists(a.layouts):
        import yaml
        layouts = (yaml.safe_load(open(a.layouts)) or {}).get('layouts', {}) or {}
    overrides = dict(kv.split('=', 1) for kv in a.set)
    runs, laps, passes, series, errors = [], [], [], {}, []
    for b in a.bags:
        try:
            R, lr, ps, se = analyse(b, layouts, a, overrides)
        except Exception as e:  # report, never invent
            errors.append(f'{b}: {e}')
            print('SKIP', b, e, file=sys.stderr)
            continue
        runs.append(R); laps += lr; passes += ps; series[R['run']] = se
        print(f"ok {R['run']}: {R['controller']} cap={R['cap_label']} laps={R['laps']} auto={R['autodrive_s']} s")
    txt = lapfiles(a.lapfiles) if a.lapfiles else []
    conds = {}
    for r in runs:
        conds.setdefault((r['cap_label'], r['layout'], r['map']), []).append(r)
    write_csv(os.path.join(a.out, 'runs.csv'), runs)
    write_csv(os.path.join(a.out, 'laps.csv'), laps)
    write_csv(os.path.join(a.out, 'avoidance_passes.csv'), passes)
    write_csv(os.path.join(a.out, 'laps_lap_analyser_txt.csv'), txt)
    write_md(os.path.join(a.out, 'summary.md'), runs, txt, conds)
    if errors:
        with open(os.path.join(a.out, 'summary.md'), 'a') as fh:
            fh.write('\n## 분석 제외 bag\n\n' + '\n'.join(f'- {e}' for e in errors) + '\n')
    if runs:
        plots(a.out, runs, series, conds)
    print('->', a.out)


if __name__ == '__main__':
    main()
