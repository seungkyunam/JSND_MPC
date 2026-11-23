"""Start at s=12.45 (real log), v=0, run 3 s: PP vs MPC under servo rate/delay variants."""
import sys, collections, numpy as np
sys.path.insert(0, '/home/jsnd/JSND_INT/tests_offline')
import lap_sim as S

OV = {}
def run(ctrl, rate_srv, delay, s0=12.45, d0=0.0, yaw0=0.0, T=3.0, verbose=False, pose_delay=0, sign=1.0, comp=0.0):
    c, p = S.build(ctrl, '3p0', OV, 50)
    conv = c.converter; dt = 0.02; L = p.get('wheelbase', 0.324)
    i = int(np.argmin(abs(S.FULL[:, 4] - s0)))
    x, y, yaw = S.FULL[i, 0], S.FULL[i, 1], S.FULL[i, 6]
    x -= d0*np.sin(yaw); y += d0*np.cos(yaw); yaw += yaw0
    v = delta = 0.0; q = collections.deque([0.0]*delay); acc = np.zeros(10)
    pq = collections.deque()
    st = collections.Counter(); first_stop = None; rows = []
    for k in range(int(T/dt)):
        s, d = [float(a[0]) for a in conv.get_frenet([x], [y])]
        wp = S.local_wpnts(s)
        c.yaw_rate = v*np.tan(delta)/L
        pq.append((x, y, yaw, v)); px, py, pyaw, pv = pq[0] if len(pq) > pose_delay else pq[0]
        if len(pq) > pose_delay: pq.popleft()
        if comp > 0 and ctrl == 'mpc':
            dl = c.curr_applied_steering
            for _ in range(int(round(comp/dt))):
                px += dt*pv*np.cos(pyaw); py += dt*pv*np.sin(pyaw); pyaw += dt*pv*np.tan(dl)/L
        out = c.main_loop('GB_TRACK', np.array([[px, py, pyaw]]), wp, pv, None, np.array([s, d, v, 0.]), acc, S.TRACK_LEN)
        spd, steer = float(out[0]), float(out[3])
        if ctrl == 'mpc':
            r = c.mpc_result; st[r.status.split(':')[0]] += 1
            if spd <= 0: c.reset_mpc_speed_command()
            if not r.success and first_stop is None and not r.status.startswith('pp_fallback'): first_stop = (round(k*dt, 2), r.status)
            c.record_applied_steering(steer)
        q.append(sign*float(np.clip(steer, -.49, .49))); cmd = q.popleft()
        delta += float(np.clip(cmd-delta, -rate_srv*dt, rate_srv*dt))
        a = float(np.clip((spd-v)/0.12, -6, 4)); vn = max(0., v+a*dt)
        x += dt*v*np.cos(yaw); y += dt*v*np.sin(yaw); yaw += dt*v*np.tan(delta)/L; v = vn
        psi = float(np.interp(s, S.FULL[:, 4], np.unwrap(S.FULL[:, 6])))
        rows.append((k*dt, v, steer, c.mpc_pp_steering if ctrl == 'mpc' else steer, d, (yaw-psi+np.pi) % (2*np.pi)-np.pi))
        if verbose and k % 5 == 0: print('t=%.2f v=%.2f steer=%+.3f pp=%+.3f d=%+.3f epsi=%+.3f' % rows[-1])
    R = np.array(rows)
    return dict(maxd=round(abs(R[:, 4]).max(), 3), maxepsi=round(abs(R[:, 5]).max(), 3),
                sat=int((abs(R[:, 2]) > .39).sum()), vmax=round(R[:, 1].max(), 2), first_stop=first_stop, st=dict(st))

if __name__ == '__main__':
    for pd, comp, rs, dl in [(6,0,6,1),(6,.06,6,1),(6,.1,6,1),(6,.1,2,5),(0,.1,6,1),(3,.1,6,1),(10,.1,6,1)]:
        print('pose_delay', pd, 'comp', comp, 'servo', rs, dl, run('mpc', rs, dl, pose_delay=pd, comp=comp))
# (appended) real Controller code path with mpc_latency_comp_s from mpc_car.yaml
