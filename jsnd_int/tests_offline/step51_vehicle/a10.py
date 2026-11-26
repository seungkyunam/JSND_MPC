from common import *
pf=np.array(D['/pf/pose/odom']); imu=np.array(D['/vesc/sensors/imu_corrected']); core=np.array(D['/vesc/sensors/core'])
t=np.arange(B[0],B[-1],0.1); y=np.unwrap(np.interp(t,pf[:,0],np.unwrap(pf[:,3])))
r_pf=np.gradient(y,t); r_imu=np.interp(t,imu[:,0],imu[:,3]); v=np.interp(t,core[:,0],core[:,1]/4614)
m=v>1.0
for lag in (0,0.05,0.1,0.15,0.2):
    ri=np.interp(t-lag,imu[:,0],imu[:,3])
    print(lag, 'corr',np.corrcoef(r_pf[m],ri[m])[0,1].round(3),'gain',(np.dot(r_pf[m],ri[m])/np.dot(ri[m],ri[m])).round(3))
