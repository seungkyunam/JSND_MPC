import sys, json, collections, numpy as np
sys.path.insert(0, '/home/jsnd/JSND_INT/tests_offline')
import lap_sim as L
L.REAL_DEADLINES = '--real' in sys.argv
args = [a for a in sys.argv[1:] if a != '--real']
noise = 0.02 if '--noise' in sys.argv else 0.0
args = [a for a in args if a != '--noise']
for spec in args:
    ov = {k: float(v) if '.' in v else int(v) for k, v in (kv.split('=') for kv in spec.split(',') if kv)}
    ov['mpc_speed_source'] = 'integrated'
    res, R = L.simulate('mpc', '5p0', ov, 50, 3, t_max=40, noise_pos=noise, noise_v=noise and 0.05, servo_delay_ticks=3)
    st = np.array(res.pop('_status_seq'))
    s = R['s']; stop = R['stop'].astype(bool)
    hp = (s > 8.5) & (s < 11.5)
    dd = np.abs(np.diff(R['steer']))
    el = R['elapsed_ms']
    budget = sum(x.startswith(('osqp', 'deadline')) for x in st)
    print(json.dumps(dict(cfg=spec or 'base', laps=res['lap_times'], stops=res['stops'], stop_ticks=res['stop_ticks'],
        hairpin_stop=int((stop & hp).sum()), lat_max=res.get('flying_lap_lat_err_max', res['lat_err_max']),
        vmax=res['max_speed'], dsteer_p99=round(float(np.percentile(dd, 99)), 3),
        steer_flip=int(np.sum(np.diff(np.sign(np.round(np.diff(R['steer']), 3))) != 0)),
        ms_p50_p99_max=[round(float(np.percentile(el, 50)), 2), round(float(np.percentile(el, 99)), 2), round(float(el.max()), 2)],
        miss_20ms=int((el > 20).sum()), budget_fail=budget,
        fails={k: v for k, v in collections.Counter(x.split(':')[0] if x not in ('solved', 'braking_recovery') else 'ok' for x in st).most_common() if k != 'ok'})), flush=True)
