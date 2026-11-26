import sys, numpy as np
sys.path.insert(0,'.')
from controller.mpc import integrated_mpc as im
from test_corner_stall import config
c=config(mpc_max_accel_mps2=5.,mpc_max_decel_mps2=5.,mpc_max_lateral_accel_mps2=4.5,mpc_max_speed_mps=5.,mpc_max_steer_rate_radps=3.2)
st=.05; n=400; s=np.arange(n)*st; a0=float(sys.argv[1]); k=np.where((s>a0)&(s<a0+3),1.0,0.)
yaw=np.r_[0.,np.cumsum(k[:-1]*st)]; x=np.r_[0.,np.cumsum(np.cos(yaw[:-1])*st)]; y=np.r_[0.,np.cumsum(np.sin(yaw[:-1])*st)]
path=np.column_stack((x,y,np.full(n,5.)))
for flag in (False,True):
    im.IntegratedMPC.BRAKING_RECOVERY_STEER_RESERVE=flag
    out=[]
    for v0 in np.arange(1.8,3.61,0.2):
        r=im.IntegratedMPC(c).solve(np.array([0.,0.,0.,v0]),path,float(sys.argv[2]))
        ok=r.controls is not None and r.success
        out.append(f"{v0:.1f}:{r.status[:22]} d0={r.steering:+.3f} a0={r.acceleration:+.2f} dmax={np.max(np.abs(r.controls[:,1])) if ok else 0:.3f}")
    print('reserve',flag); print('\n'.join(out))
