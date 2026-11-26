import pickle,sys,collections,numpy as np,json,bisect
D=pickle.load(open(sys.argv[1],'rb')); W=pickle.load(open(sys.argv[1]+'.w','rb')); t0,A0,A1=W['t0'],W['A0'],W['A1']
ina=lambda tb:A0<=tb<=A1
fr=np.array(D['fr']); ft=fr[:,0]
def at(tb):
    i=min(bisect.bisect(ft,tb),len(ft)-1); return fr[i]
dg=[d for d in D['diag'] if ina(d['tb'])]
# episodes by SM state
eps=[];cur=None
for d in dg:
    if cur is None or d['state']!=cur[0]:
        if cur: eps.append(cur)
        cur=[d['state'],d['tb'],d['tb'],collections.Counter()]
    cur[2]=d['tb']; cur[3][d['status']]+=1
eps.append(cur)
dbg=D['dbg']; dt=[x[0] for x in dbg]
for st,a,b,c in eps:
    fa,fb=at(a),at(b)
    seg=fr[(ft>=a)&(ft<=b)]
    j=dbg[min(bisect.bisect(dt,a),len(dbg)-1)]
    sf=json.loads(j[4]) if j[4]!='null' else None; pdg=json.loads(j[5]) if j[5]!='null' else None
    obs=[(o.get('gap'),o.get('d'),o.get('branch'),o.get('blocked')) for o in (sf or {}).get('obs',[])] if sf else None
    bad={k:v for k,v in c.items() if k!='solved'}
    print(f"{a-t0:6.2f}-{b-t0:6.2f} {st:9s} s {fa[1]:5.2f}->{fb[1]:5.2f} d[{seg[:,2].min():+.2f},{seg[:,2].max():+.2f}] v[{seg[:,3].min():.2f},{seg[:,3].mean():.2f}] bad={bad} obs={obs} pd={pdg.get('reason') if pdg else None}")
