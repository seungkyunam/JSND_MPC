import pickle,sys,numpy as np,bisect
D=pickle.load(open(sys.argv[1],'rb')); W=pickle.load(open(sys.argv[1]+'.w','rb')); t0,A0,A1=W['t0'],W['A0'],W['A1']
G=np.array(pickle.load(open(sys.argv[2],'rb')))
for s in (0.9,1.6,2.5,3.4,4.0,5.5,7.9,12.6): i=np.argmin(abs(G[:,0]-s)); print(f'track s{s}: d_right {G[i,3]:.2f} d_left {G[i,4]:.2f} kappa {G[i,6]:+.2f}')
fr=D['fr']; ft=[x[0] for x in fr]
ev=[]
for s in D['scan']:
    if not (A0<=s[0]<=A1): continue
    r=np.array(s[4]); r[~np.isfinite(r)]=99; r[r<0.05]=99; i=r.argmin()
    ev.append((s[0]-t0,r[i],s[2]+i*s[3]))
ev=np.array(ev); print('scan min range overall p1/min',np.percentile(ev[:,1],1).round(3),ev[:,1].min().round(3))
# episodes with min<0.25
m=ev[:,1]<0.25; g=[]
for t,rr,a in ev[m]:
    if g and t-g[-1][1]<0.3: g[-1][1]=t; g[-1][2]=min(g[-1][2],rr); g[-1][3].append(a)
    else: g.append([t,t,rr,[a]])
for a,b,rr,an in g:
    f=fr[bisect.bisect(ft,a+t0)]; print(f'  <0.25m {a:6.2f}-{b:6.2f} min {rr:.3f} m ang {np.degrees(np.median(an)):+.0f}deg  s {f[1]:.2f} d {f[2]:+.2f} v {f[3]:.2f}')
# idle (before autodrive) baseline: min range while stationary at start
