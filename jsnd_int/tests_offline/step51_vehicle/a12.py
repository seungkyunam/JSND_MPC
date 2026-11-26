from common import *
import pickle
G=np.array(pickle.load(open('gw.pkl','rb')))
dg=D['/controller/mpc_diagnostics']; core=np.array(D['/vesc/sensors/core']); cmd=np.array(D['/vesc/ackermann_cmd'])
imu=np.array(D['/vesc/sensors/imu_corrected'])
n=lambda t,ts:min(np.searchsorted(ts,t),len(ts)-1)
print('raceline vx max',G[:,5].max().round(2),'at s',G[G[:,5].argmax(),0].round(1),' straight s5-8 vx max',G[(G[:,0]>5)&(G[:,0]<8),5].max().round(2),' s11-14',G[(G[:,0]>11),5].max().round(2))
rows=[]
for ts,j in dg:
    if not B[0]<=ts<B[-1] or 'optimal_accel_mps2' not in j: continue
    s=s_at(ts)[0]; lap=lap_of(ts)
    vref=np.interp(s,G[:,0],G[:,5])
    rows.append((lap,s,j['measured_mps'],j['optimal_accel_mps2'],j['applied_accel_mps2'],j['command_unclipped_mps'],j['command_clipped_mps'],j['lead_limit_mps'],j['jerk_limited'],j['lead_limited'],j['plan_limited'],j['predicted_peak_mps'],vref,ts,j['controller_speed_mps'],j.get('guarded_speed_mps') or 0))
R=np.array(rows,dtype=float)
def straight(s): return ((s>5.0)&(s<7.8))|(s>11.0)|(s<0.8)
for name,sel in [('fast6,11',np.isin(R[:,0],[6,11])),('slow8,12,13,14',np.isin(R[:,0],[8,12,13,14])),('all',R[:,0]>0)]:
    m=sel&straight(R[:,1])&(R[:,2]>0.5)
    acc=m&(R[:,3]>0.2)
    print(f"[{name}] straight ticks={m.sum()} accelerating={acc.sum()}")
    print(f"  opt_accel p50/p95={np.percentile(R[acc,3],[50,95]).round(2)} applied p50={np.median(R[acc,4]):.2f}  cmd-meas(unclip) p50={np.median(R[acc,5]-R[acc,2]):.2f} (clip) p50={np.median(R[acc,6]-R[acc,2]):.2f} final(guarded)-meas p50={np.median(R[acc,15]-R[acc,2]):.2f}")
    print(f"  lead_limited={R[acc,9].mean():.2f} jerk_limited={R[acc,8].mean():.2f} plan_limited={R[acc,10].mean():.2f}  vref(raceline)-meas p50={np.median(R[acc,12]-R[acc,2]):.2f}  pred_peak-meas p50={np.median(R[acc,11]-R[acc,2]):.2f}")
# real accel from wheel speed on straights per lap
print('lap vmax_whl  a_real(p50 when accelerating, s5-7.8)  a_real(s11-14)  a_imu_x p50')
for lap in range(1,15):
    a,b=B[lap-1],B[lap]; m=(core[:,0]>=a)&(core[:,0]<b); t=core[m,0]; v=core[m,1]/4614
    vs=np.convolve(v,np.ones(5)/5,'same'); acc=np.gradient(vs,t)
    ss=np.array([s_at(x)[0] for x in t])
    out=[]
    for lo,hi in((5.0,7.8),(11.0,13.9)):
        k=(ss>lo)&(ss<hi)&(acc>0.3)&(v>0.6)
        out.append(np.median(acc[k]) if k.sum()>3 else np.nan)
    mi=(imu[:,0]>=a)&(imu[:,0]<b)
    print(f"L{lap:2d} {v.max():.2f} {out[0]:.2f} {out[1]:.2f}")
