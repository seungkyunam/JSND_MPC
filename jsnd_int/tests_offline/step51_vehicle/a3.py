from common import *
import sys
diag=D['/controller/mpc_diagnostics']; sm=D['/state_machine']
smt=np.array([x[0] for x in sm])
cmd=np.array([(x[0],x[1]) for x in D['/vesc/ackermann_cmd']])
def key(x):
    s=x[1]['status']; st=sm[min(np.searchsorted(smt,x[0]),len(sm)-1)][1]
    s=re.sub(r'_late|_inexact_verified','',s)
    if s in('solved','tracking local path'): s='ok'
    return st+'|'+s
for lap in map(int,sys.argv[1:]):
    a,b=B[lap-1],B[lap]; print(f'==== L{lap}')
    seg=None
    for x in diag:
        if not a<=x[0]<b: continue
        k=key(x)
        if seg is None or seg[0]!=k:
            if seg: 
                s0,d0,_=s_at(seg[1]); s1,_,_=s_at(seg[2])
                print(f"{seg[1]-a:6.2f}-{seg[2]-a:6.2f} n={seg[3]:3d} s={s0:5.2f}->{s1:5.2f} d={d0:+.2f} v={seg[4]:.2f} cmd={seg[5]:.2f} {seg[0]}")
            seg=[k,x[0],x[0],0,(x[1].get('measured_mps') or -1),x[1].get('guarded_speed_mps') or 0]
        seg[2]=x[0]; seg[3]+=1
    for t in ttc:
        if a<=t<b: s,d,v=s_at(t); print(f"  TTC @{t-a:6.2f} s={s:.2f} d={d:+.2f} vs={v:.2f}")
