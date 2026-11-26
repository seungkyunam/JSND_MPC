import pickle,sys,numpy as np,bisect
D=pickle.load(open(sys.argv[1],'rb')); W=pickle.load(open(sys.argv[1]+'.w','rb')); t0,A0,A1=W['t0'],W['A0'],W['A1']
fr=np.array([x for x in D['fr'] if A0<=x[0]<=A1]); sm=D['sm']; smt=[x[0] for x in sm]
obs=D['obs']; ot=[x[0] for x in obs]
m=(fr[:,1]>7.5)&(fr[:,1]<8.2)
idx=np.where(m)[0]; grp=np.split(idx,np.where(np.diff(idx)>5)[0]+1)
for g in grp:
    if len(g)==0: continue
    a,b=fr[g[0],0],fr[g[-1],0]
    o8=[o for tb,L in obs if a-1<=tb<=b for o in L if 7.6<o[1]+0.1 and o[0]<8.2]
    st=set(sm[bisect.bisect(smt,t)-1][1] for t in fr[g,0])
    print(f'{a-t0:6.2f}-{b-t0:6.2f} d[{fr[g,2].min():+.2f},{fr[g,2].max():+.2f}] v {fr[g,3].mean():.2f} state {st} obs@s8 검출 {len(o8)}회', (f'dR {min(o[2] for o in o8):.2f}' if o8 else ''))
