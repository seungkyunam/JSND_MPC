import sys, json, numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/',1)[0])
import lap_sim49 as ls
def run(ov, pd=7, sd=3, laps=3, seed=0, ny=0.02, npos=0.02, nv=0.05, real=False, hist=False, miss=0.0, bias=0.0, undershoot=False):
    from controller.mpc.integrated_mpc import IntegratedMPC
    if not hasattr(IntegratedMPC, '_orig_solve'): IntegratedMPC._orig_solve = IntegratedMPC._solve
    mr = np.random.default_rng(seed+100); burst = [0]
    def _solve(self, *a, **k):
        r = IntegratedMPC._orig_solve(self, *a, **k)
        if burst[0] <= 0 and mr.random() < miss: burst[0] = int(mr.integers(1, 9))
        if burst[0] > 0 and r.success:
            burst[0] -= 1; self.plan = getattr(self, '_last_plan', None) if r.success else self.plan
            return self._fail('deadline_exceeded', a[2], None)
        burst[0] = max(0, burst[0]-1); self._last_plan = self.plan
        return r
    IntegratedMPC._solve = _solve
    ls.BIAS = bias
    ls.REAL_DEADLINES = real
    ls.HISTORY = hist; ls.SIMT[0] = 0.0
    ls.VESC_TAU = 0.12
    ov = dict({'mpc_speed_source':'integrated'}, **ov)
    res, R = ls.simulate('mpc', '5p0', ov, 50, laps, t_max=40, servo_delay_ticks=sd, noise_pos=npos, noise_v=nv,
                         seed=seed, pose_delay_ticks=pd, noise_yaw=ny)
    res.pop('_status_seq')
    s = R['s']; L = 0.324
    kap = np.interp(s, ls.FULL[:,4], ls.FULL[:,5])
    ideal = np.arctan(L*kap)
    st, yr, v = R['steer'], R['yawrate'], R['v']
    m = R['t'] > 3.0
    dst = np.diff(st[m]); sig = np.sign(dst[np.abs(dst) > 0.01])
    res['steer_reversals_per_s'] = round(float(np.sum(sig[1:] != sig[:-1]) / (m.sum()/50)), 2)
    acc = np.diff(R['cmd'][m]); sa = np.sign(acc[np.abs(acc) > 0.02])
    res['accel_flips_per_s'] = round(float(np.sum(sa[1:] != sa[:-1]) / (m.sum()/50)), 2)
    # corner phases by |kappa| and its slope along s
    ak = np.abs(kap); dk = np.gradient(ak)
    corner = ak > 0.4
    ph = {'entry': corner & (dk > 0.002), 'mid': corner & (np.abs(dk) <= 0.002), 'exit': corner & (dk < -0.002)}
    out = {}
    for n_, msk in ph.items():
        msk = msk & m
        if msk.sum() < 5: continue
        out[n_] = {'n': int(msk.sum()), 'steer_minus_ideal': round(float(np.mean(np.sign(kap[msk])*(st[msk]-ideal[msk]))), 3),
                   'yawrate_over_ideal': round(float(np.mean(yr[msk]/np.maximum(v[msk],0.1)/np.where(np.abs(kap[msk])>1e-3,kap[msk],1e-3))), 2),
                   'd_absmean': round(float(np.mean(np.abs(R['d'][msk]))), 3)}
    res['phases'] = out
    # lag cmd curvature -> actual yaw rate/v
    c1 = np.tan(st[m])/L; c2 = yr[m]/np.maximum(v[m], .1)
    lags = range(0, 20); cc = [np.corrcoef(c1[:len(c1)-k], c2[k:])[0,1] for k in lags]
    res['cmd_to_yaw_lag_ms'] = 20*int(np.argmax(cc))
    lags2 = range(-20, 21); ci = np.tan(ideal[m])/L
    cc2 = [np.corrcoef(ci[max(0,-k):len(ci)-max(0,k)], c1[max(0,k):len(c1)-max(0,-k)])[0,1] for k in lags2]
    res['cmd_vs_path_lead_ms'] = -20*list(lags2)[int(np.argmax(cc2))]  # >0: command leads path curvature
    return res, R
if __name__ == '__main__':
    ov = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    res, _ = run(ov, **kw)
    print(json.dumps(res, ensure_ascii=False))
