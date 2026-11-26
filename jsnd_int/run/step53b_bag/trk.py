import pickle,sys,numpy as np,bisect,collections
D=pickle.load(open(sys.argv[1],'rb')); W=pickle.load(open(sys.argv[1]+'.w','rb')); t0,A0,A1=W['t0'],W['A0'],W['A1']
pf=np.array(D['pf']); pt=pf[:,0]; bs=D['bs']; bt=[x[0] for x in bs]
dg=[d for d in D['diag'] if A0<=d['tb']<=A1]
E=collections.defaultdict(list)
for d in dg:
    i=bisect.bisect(bt,d['tb'])-1; j=bisect.bisect(pt,d['tb'])-1
    if i<0 or j<0: continue
    W_=np.array(bs[i][1])
    if len(W_)<2: continue
    dist=np.hypot(W_[:,2]-pf[j,2],W_[:,3]-pf[j,3]); E[d['state']].append(dist.min())
for k,v in E.items(): v=np.array(v); print(f'{k}: PF↔local_wpnts 최근접거리 p50 {np.median(v):.3f} p95 {np.percentile(v,95):.3f} max {v.max():.3f} n {len(v)}')
hc=np.array([(tb,v,s) for tb,v,s in D['hcmd'] if A0<=tb<=A1]); st=hc[:,2]
ds=np.sign(np.diff(st)); ds=ds[ds!=0]; rev=(np.diff(ds)!=0).sum()
print('steer rate |d|/dt p95 %.2f rad/s, 방향반전 %d회(%.1f/s), 포화>=0.395 %.1f%%'%(np.percentile(abs(np.diff(st))/np.diff(hc[:,0]).clip(1e-3),95),rev,rev/(A1-A0),100*(abs(st)>=0.395).mean()))
# saturation by state
ss=collections.defaultdict(list)
for d in dg: ss[d['state']].append(abs(d.get('steering_rad') or 0)>=0.395)
print({k:round(100*np.mean(v),1) for k,v in ss.items()})
# speed: controller_speed vs measured
cs=np.array([d['controller_speed_mps'] or 0 for d in dg]); ms=np.array([d['measured_mps'] for d in dg])
print('cmd<0.05 비율 %.1f%%, |cmd-meas| p50 %.2f'%(100*(cs<0.05).mean(),np.median(abs(cs-ms))))
nonsol=[d for d in dg if d['status'] not in('solved','solved_late','solved_inexact_verified')]
print('non-solved ticks',len(nonsol),'of',len(dg), 'cmd0 among them', sum((d['controller_speed_mps'] or 0)<0.05 for d in nonsol))
