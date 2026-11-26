from common import *
pf=np.array(D['/pf/pose/odom']); core=np.array(D['/vesc/sensors/core']); imu=np.array(D['/vesc/sensors/imu_corrected'])
wv=np.interp(pf[:,0],core[:,0],core[:,1]/4614)
wz=np.interp(pf[:,0],imu[:,0],imu[:,3])
W=0.5
print("lap  time  stall(s)  jump>0.25m  yawErr>0.3(s)  maxd_hair  stall_s_locs")
for lap in range(1,15):
    a,b=B[lap-1],B[lap]; idx=np.where((pf[:,0]>=a)&(pf[:,0]<b))[0]
    stall=0; locs=[]; jumps=0; yerr=0
    for i in idx[::5]:
        j=np.searchsorted(pf[:,0],pf[i,0]+W)
        if j>=len(pf): break
        seg=slice(i,j)
        wd=np.trapz(np.abs(wv[seg]),pf[seg,0]); pd=np.hypot(*(pf[j,1:3]-pf[i,1:3]))
        if wd>0.4 and pd<0.4*wd: stall+=0.125 ; locs.append(round(s_at(pf[i,0])[0],1))
        dyaw_imu=np.trapz(wz[seg],pf[seg,0]); dyaw_pf=np.angle(np.exp(1j*(pf[j,3]-pf[i,3])))
        if abs(dyaw_imu-dyaw_pf)>0.3: yerr+=0.125
    st=np.diff(pf[idx,1:3],axis=0); dt=np.diff(pf[idx,0]); step=np.hypot(st[:,0],st[:,1])
    jumps=int(((step-np.abs(wv[idx[1:]])*dt)>0.25).sum())
    hs=[abs(s_at(t)[1]) for t in pf[idx,0] if 2.8<=s_at(t)[0]<=4.2]
    print(f"L{lap:2d} {b-a:6.2f} {stall:6.2f} {jumps:4d} {yerr:6.2f} {max(hs) if hs else 0:5.2f}  {sorted(set(locs))[:12]}")
