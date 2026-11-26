from common import *
import pickle
G=np.array(pickle.load(open('gw.pkl','rb')))
pf=np.array(D['/pf/pose/odom']); core=np.array(D['/vesc/sensors/core']); imu=np.array(D['/vesc/sensors/imu_corrected'])
dg=D['/controller/mpc_diagnostics']; dgt=np.array([x[0] for x in dg])
n=lambda t,ts:min(np.searchsorted(ts,t),len(ts)-1)
print("lap  v@s7.5  vmin8-10  brkrec_ticks  brkrec_steer0  hdgSTOP  firstfail_s  yawPF-IMU_max(7.5-9.5)  PFstall8-10")
for lap in range(1,15):
    a,b=B[lap-1],B[lap]
    ts=pf[(pf[:,0]>=a)&(pf[:,0]<b),0]
    ss=np.array([s_at(t)[0] for t in ts])
    m=(ss>=7.4)&(ss<7.6); v75=np.interp(ts[m],core[:,0],core[:,1]/4614).mean() if m.any() else np.nan
    # first pass of s in [7.5,9.8] only
    idx=np.where((ss>=7.5)&(ss<9.8))[0]
    if len(idx)==0: print(lap,'no'); continue
    t_in=ts[idx[0]]
    # window: until s>10 first time after t_in
    after=np.where((ts>t_in)&(ss>10.0))[0]; t_out=ts[after[0]] if len(after) else b
    seg=[x for x in dg if t_in<=x[0]<t_out]
    br=[x for x in seg if x[1]['status'].startswith('braking_recovery')]
    br0=sum(1 for x in br if abs(x[1].get('steering_rad') or 0)<0.02)
    hs=sum(1 for x in seg if 'heading' in x[1]['status'])
    fail=[x for x in seg if not x[1]['status'].startswith(('solved','braking_rec','plan_cont'))]
    fs=s_at(fail[0][0])[0] if fail else None
    # yaw discrepancy over 0.5 s windows
    mx=0
    for t in np.arange(t_in,min(t_out,t_in+4),0.1):
        i,j=n(t,pf[:,0]),n(t+0.5,pf[:,0]); dpf=np.angle(np.exp(1j*(pf[j,3]-pf[i,3])))
        mm=(imu[:,0]>=pf[i,0])&(imu[:,0]<pf[j,0]); dimu=np.trapz(imu[mm,3],imu[mm,0]); mx=max(mx,abs(dpf-dimu))
    vmin=np.interp(np.linspace(t_in,t_out,50),core[:,0],core[:,1]/4614).min()
    print(f"L{lap:2d} {v75:5.2f} {vmin:5.2f} {len(br):4d} {br0:4d} {hs:4d} {fs if fs is None else round(fs,2)} {mx:5.2f}  dur={t_out-t_in:5.2f}s")
