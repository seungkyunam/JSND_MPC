import pickle,sys,numpy as np,bisect,math,collections
D=pickle.load(open(sys.argv[1],'rb')); W=pickle.load(open(sys.argv[1]+'.w','rb')); t0,A0,A1=W['t0'],W['A0'],W['A1']
G=np.array(pickle.load(open(sys.argv[2],'rb')))
pf=np.array([x for x in D['pf'] if A0<=x[0]<=A1]); od=np.array([x for x in D['od'] if A0<=x[0]<=A1])
lat=(pf[:,0]-pf[:,1])*1e3; print('PF rate %.1f Hz, gap p99 %.0f max %.0f ms; header->bag latency p50/p95/max %s ms'%(len(pf)/(A1-A0),np.percentile(np.diff(pf[:,0])*1e3,99),np.diff(pf[:,0]).max()*1e3,np.percentile(lat,[50,95,100]).round(0)))
lo=(od[:,0]-od[:,1])*1e3; print('car_state/odom latency p50/p95/max',np.percentile(lo,[50,95,100]).round(0))
# PF step jumps vs wheel speed
core=np.array(D['core']); ct=core[:,0]
dpf=np.hypot(np.diff(pf[:,2]),np.diff(pf[:,3])); dtp=np.diff(pf[:,0])
wv=np.interp(pf[1:,0],ct,abs(core[:,1])/4614.)
ex=dpf-wv*dtp
print('PF step excess over wheel: p95 %.3f m, >0.10 m: %d'%(np.percentile(ex,95),(ex>0.10).sum()))
for i in np.where(ex>0.10)[0][:15]: print(f'   t={pf[i+1,0]-t0:.2f} jump={dpf[i]:.2f}m dt={dtp[i]*1e3:.0f}ms wheel={wv[i]:.2f}')
# PF stall: wheel>0.4 but PF moved < 0.02 m over 0.5 s
k=0;stalls=[]
for i in range(len(pf)):
    j=bisect.bisect(pf[:,0],pf[i,0]+0.5)
    if j>=len(pf): break
    if np.hypot(pf[j,2]-pf[i,2],pf[j,3]-pf[i,3])<0.05 and np.interp(pf[i,0],ct,abs(core[:,1])/4614.)>0.4: stalls.append(pf[i,0]-t0)
if stalls:
    g=[[stalls[0],stalls[0]]]
    for x in stalls[1:]:
        if x-g[-1][1]<0.3: g[-1][1]=x
        else: g.append([x,x])
    print('PF stalls (wheel>0.4, PF<5cm/0.5s):',[(round(a,2),round(b+0.5,2)) for a,b in g])
else: print('PF stalls: none')
# yaw diff pf vs EKF
yo=np.interp(pf[:,0],od[:,0],np.unwrap(od[:,4])); dy=(pf[:,4]-yo+np.pi)%(2*np.pi)-np.pi
dx=np.hypot(pf[:,2]-np.interp(pf[:,0],od[:,0],od[:,2]),pf[:,3]-np.interp(pf[:,0],od[:,0],od[:,3]))
print('PF vs car_state/odom pos diff p50/p95/max',np.percentile(dx,[50,95,100]).round(3),'yaw diff p95',np.percentile(abs(dy),95).round(3))
# wheel vs frenet vs
fr=np.array([x for x in D['fr'] if A0<=x[0]<=A1]); wvf=np.interp(fr[:,0],ct,core[:,1]/4614.)
print('wheel speed min/max',wvf.min().round(2),wvf.max().round(2),'reverse(<-0.05) frac',(wvf<-0.05).mean().round(3))
# s backward while wheel forward
fs=np.unwrap(fr[:,1],period=G[-1,0]+ (G[1,0]-G[0,0]))
