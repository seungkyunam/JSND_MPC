from common import *
import pickle
G=np.array(pickle.load(open('gw.pkl','rb')))
core=np.array(D['/vesc/sensors/core']); cmd=np.array(D['/vesc/ackermann_cmd']); pf=np.array(D['/pf/pose/odom']); imu=np.array(D['/vesc/sensors/imu_corrected'])
m=(core[:,0]>B[0])&(core[:,0]<B[-1]); t=core[m,0]; v=core[m,1]/4614; vs=np.convolve(v,np.ones(5)/5,'same'); a=np.gradient(vs,t)
c=np.interp(t,cmd[:,0],cmd[:,1]); e=c-v; k=(v>0.6)&(e>0.2)&(e<1.0)
for lag in (0,0.04,0.08,0.12):
    ce=np.interp(t-lag,cmd[:,0],cmd[:,1])-v; kk=(v>0.6)&(ce>0.2)&(ce<1.0)
    tau=np.dot(ce[kk],ce[kk])/np.dot(ce[kk],a[kk]); print(f'lag{lag} tau={tau:.2f} corr={np.corrcoef(ce[kk],a[kk])[0,1]:.2f}')
acc=k&(a>0.3); print('accelerating: motor current p50/p95',np.percentile(core[m][acc,2],[50,95]).round(1),'duty p95',np.percentile(core[m][acc,3],95).round(2))
# steering delay: cmd steer vs geometric need atan(L*kappa) at PF s; and vs IMU yaw-rate implied
L=0.33
ts=np.arange(B[0],B[-1],0.02)
st=np.interp(ts,cmd[:,0],cmd[:,2]); vv=np.interp(ts,core[:,0],core[:,1]/4614)
fr=np.array(D['/car_state/odom_frenet']); s=np.interp(ts,fr[:,0],fr[:,1])
kap=np.interp(s,G[:,0],G[:,4]); need=np.arctan(L*kap)
wz=np.interp(ts,imu[:,0],imu[:,3]); imp=np.arctan(L*wz/np.maximum(vv,0.3))
def lagbest(x,y,mask):
    best=None
    for sh in range(-25,26):
        if sh>=0: xa,ya,ma=x[:len(x)-sh],y[sh:],mask[:len(x)-sh]
        else: xa,ya,ma=x[-sh:],y[:sh],mask[-sh:]
        r=np.corrcoef(xa[ma],ya[ma])[0,1]
        if best is None or r>best[1]: best=(sh*0.02,r)
    return best
for name,laps in(('fast6,11',[6,11]),('slow8,12,13,14',[8,12,13,14]),('all',range(1,15))):
    msk=np.zeros(len(ts),bool)
    for l in laps: msk|=(ts>=B[l-1])&(ts<B[l])
    msk&=vv>1.0
    print(name,'steer vs raceline-need: (lag s, corr)',np.round(lagbest(need,st,msk),2),' imu-implied vs steer:',np.round(lagbest(st,imp,msk),2))
# tracking error: PF xy to raceline polyline
P=G[:,1:3]
def dist(x,y):
    dd=np.hypot(P[:,0]-x,P[:,1]-y); return dd.min()
print('lap  |e|p50  p95  max   (PF to raceline, v>1)')
for l in range(1,15):
    mm=(pf[:,0]>=B[l-1])&(pf[:,0]<B[l]); vv2=np.interp(pf[mm,0],core[:,0],core[:,1]/4614)
    e=np.array([dist(x,y) for x,y in pf[mm][:,1:3]])[vv2>1.0]
    print(f"L{l:2d} {np.median(e):.3f} {np.percentile(e,95):.3f} {e.max():.3f}")
