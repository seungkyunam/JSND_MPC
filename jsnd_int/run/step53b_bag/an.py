import pickle,sys,collections,numpy as np,json,math
D=pickle.load(open(sys.argv[1],'rb')); t0=D['od'][0][0]
H=[(tb,v) for tb,v in D['host']]
aut=[tb for tb,v in H if v=='autodrive']; A0,A1=min(aut),max(aut); print(f'autodrive {A0-t0:.2f}-{A1-t0:.2f} ({A1-A0:.1f}s)')
ina=lambda tb:A0<=tb<=A1
fr=np.array([x for x in D['fr'] if ina(x[0])])
# laps
s=fr[:,1]; wr=np.where(np.diff(s)<-10)[0]
lt=[fr[i+1,0]-t0 for i in wr]; print('lap cross t',np.round(lt,2),'lap times',np.round(np.diff(lt),2))
# obstacles static clusters
ob=[(tb,o) for tb,L in D['obs'] if ina(tb) for o in L]
cl=collections.defaultdict(list)
for tb,o in ob: cl[(round((o[0]+o[1])/2),o[6])].append(o)
for k,v in sorted(cl.items()):
    v=np.array([x[:6] for x in v]); print('obs s~',k,'n',len(v),'s',v[:,0].min().round(2),v[:,1].max().round(2),'dR',v[:,2].min().round(3),v[:,2].max().round(3),'dL',v[:,3].min().round(3),v[:,3].max().round(3))
# diag in autodrive
dg=[d for d in D['diag'] if ina(d['tb'])]
print('status',collections.Counter(d['status'] for d in dg).most_common())
print('state',collections.Counter(d['state'] for d in dg).most_common())
el=np.array([v for tb,v in D['elap'] if ina(tb)]); per=np.array([v for tb,v in D['per'] if ina(tb)])
print('elapsed ms p50/p95/p99/max',np.percentile(el,[50,95,99,100]).round(1),'>20ms',(el>20).sum(),'period p50/p99/max',np.percentile(per,[50,99,100]).round(1),'rate',len(el)/(A1-A0))
# speed
cmd=np.array([(tb,v,st) for tb,v,st in D['cmd'] if ina(tb)])
hc=np.array([(tb,v,st) for tb,v,st in D['hcmd'] if ina(tb)])
print('final cmd speed max',cmd[:,1].max(),'steer |max|',abs(cmd[:,2]).max(),'sat>=0.395',(abs(hc[:,2])>=0.395).mean().round(3))
vs=fr[:,3]; print('vs p50/p90/max',np.percentile(vs,[50,90,100]).round(2),'frac vs<0.1',(vs<0.1).mean().round(3))
# stops: intervals vs<0.1 longer than 0.3s
st=None
for (tb,ss,dd,v) in fr:
    if v<0.15 and st is None: st=(tb,ss)
    if v>=0.15 and st is not None:
        if tb-st[0]>0.3: print(f'  slow/stop {st[0]-t0:.2f}-{tb-t0:.2f} ({tb-st[0]:.2f}s) s={st[1]:.2f}')
        st=None
pickle.dump(dict(A0=A0,A1=A1,t0=t0),open(sys.argv[1]+'.w','wb'))
