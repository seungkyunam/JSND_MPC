from common import *
import sys,pickle
G=np.array(pickle.load(open('gw.pkl','rb')))
lap=int(sys.argv[1]); t0,t1,dt=map(float,sys.argv[2:5]); a=B[lap-1]
pf=np.array(D['/pf/pose/odom']); core=np.array(D['/vesc/sensors/core']); imu=np.array(D['/vesc/sensors/imu_corrected']); cmd=np.array(D['/vesc/ackermann_cmd'])
dg=D['/controller/mpc_diagnostics']; dgt=np.array([x[0] for x in dg])
n=lambda t,ts:min(np.searchsorted(ts,t),len(ts)-1)
i0=n(a+t0,imu[:,0]); yimu=None
print("  t   whl  pfspd  cmdv  steer  pfyaw  imuyaw  nearest_psi  hdgErr  s     d    status")
prev=None
for t in np.arange(a+t0,a+t1,dt):
    p=pf[n(t,pf[:,0])]
    k=np.argmin(np.hypot(G[:,1]-p[1],G[:,2]-p[2])); psi=G[k,3]
    if yimu is None: yimu=p[3]; ti=t
    else:
        m=(imu[:,0]>=ti)&(imu[:,0]<t); yimu+=np.trapz(imu[m,3],imu[m,0]); ti=t
    sp=0 if prev is None else np.hypot(*(p[1:3]-prev[1:3]))/max(p[0]-prev[0],1e-3); prev=p
    c=cmd[n(t,cmd[:,0])]; s,d,_=s_at(t)
    print(f"{t-a:5.2f} {core[n(t,core[:,0]),1]/4614:5.2f} {sp:5.2f} {c[1]:5.2f} {c[2]:+5.2f} {p[3]:+5.2f} {np.angle(np.exp(1j*yimu)):+5.2f}  {psi:+5.2f}(s{G[k,0]:.1f}) {np.angle(np.exp(1j*(p[3]-psi))):+5.2f} {s:5.2f} {d:+5.2f} {dg[n(t,dgt)][1]['status'][:30]}")
