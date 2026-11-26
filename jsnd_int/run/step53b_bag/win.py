import pickle,sys,numpy as np,bisect,math
D=pickle.load(open(sys.argv[1],'rb')); W=pickle.load(open(sys.argv[1]+'.w','rb')); t0=W['t0']
a,b,step=float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4])
def near(L,tb,key=lambda x:x[0]):
    ks=[key(x) for x in L]; i=max(0,min(bisect.bisect(ks,tb)-1,len(L)-1)); return L[i]
core=D['core']; pf=D['pf']; fr=D['fr']; cmd=D['cmd']; dg=D['diag']; sc=D['scan']
for t in np.arange(a,b,step):
    tb=t0+t; c=near(core,tb); p=near(pf,tb); f=near(fr,tb); k=near(cmd,tb); g=near(dg,tb,lambda x:x['tb']); s=near(sc,tb)
    r=np.array(s[4]); r[~np.isfinite(r)]=99; r[r<0.05]=99; ang=s[2]+np.arange(len(r))*s[3]
    fm=r[abs(ang)<0.5].min(); L=r[(ang>0.5)&(ang<2.0)].min(); R=r[(ang<-0.5)&(ang>-2.0)].min()
    print(f"{t:6.2f} whl {c[1]/4614:5.2f} I {c[2]:5.1f} | pf {p[2]:5.2f},{p[3]:5.2f} s {f[1]:5.2f} d {f[2]:+.2f} vs {f[3]:5.2f} | cmd v {k[1]:4.2f} st {k[2]:+.3f} | {g['state'][:4]} {g['status'][:38]:38s} cap {g.get('trailing_cap_mps') or 0:.2f} | scan F {fm:.2f} L {L:.2f} R {R:.2f}")
