from common import *
sd=D['/state_machine/debug']; dg=D['/controller/mpc_diagnostics']; dgt=np.array([x[0] for x in dg])
n=lambda t,ts:min(np.searchsorted(ts,t),len(ts)-1)
H=collections.Counter(); gaps=[]; behind=0; tot=0; wt=collections.Counter(); close=collections.Counter()
for ts,q in sd:
    if not B[0]<=ts<B[-1] or q['state']!='TRAILING': continue
    tot+=1; s=q['cur_s']; H[int(s)]+=1
    obs=(q['gb'] or {}).get('obs',[])
    g=[o['gap'] for o in obs]; gaps+=g
    if g and min(g)>6.95: behind+=1
    close[q['close_to_raceline']]+=1
    st=dg[n(ts,dgt)][1]['status']
    if st=='trailing_without_target': wt[int(s)]+=1
print('TRAILING ticks',tot,'by s',sorted(H.items()))
print('trailing_without_target by s',sorted(wt.items()))
print('close_to_raceline during TRAILING',close)
print('gb gap>half-lap ticks',behind,' gap pct',np.percentile(gaps,[5,50,95]).round(2) if gaps else None, 'max',max(gaps))
