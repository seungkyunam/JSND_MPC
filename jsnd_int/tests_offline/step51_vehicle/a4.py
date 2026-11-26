from common import *
import sys
lap=int(sys.argv[1]); t0,t1,dt=map(float,sys.argv[2:5]); a=B[lap-1]
def near(t,ts): return min(np.searchsorted(ts,t),len(ts)-1)
core=np.array(D['/vesc/sensors/core']); cmd=np.array(D['/vesc/ackermann_cmd']); hl=np.array(D['/vesc/high_level/ackermann_cmd'])
pf=np.array(D['/pf/pose/odom']); co=np.array(D['/car_state/odom'])
dg=D['/controller/mpc_diagnostics']; dgt=np.array([x[0] for x in dg])
sd=D['/state_machine/debug']; sdt=np.array([x[0] for x in sd])
ob=D['/tracking/obstacles']; obt=np.array([x[0] for x in ob])
es=D['/simple_mux/estop_state']; est=np.array([x[0] for x in es])
print(" t    whl  cmd  hl   pfx   pfy  pfyaw  s     d     state    status/gb                                     obs")
for t in np.arange(a+t0,a+t1,dt):
    c=core[near(t,core[:,0])]; k=cmd[near(t,cmd[:,0])]; h=hl[near(t,hl[:,0])]; p=pf[near(t,pf[:,0])]
    s,d,vs=s_at(t); g=dg[near(t,dgt)][1]; q=sd[near(t,sdt)][1]; o=ob[near(t,obt)][1]
    gb=q['gb']['obs'] if q.get('gb') else []
    gbs=';'.join(f"{x['id']}g{x['gap']}d{x['d']}{'B' if x['blocked'] else ''}" for x in gb)
    print(f"{t-a:5.2f} {c[1]/4614:4.2f} {k[1]:4.2f} {h[1]:4.2f} {p[1]:5.2f} {p[2]:5.2f} {p[3]:+5.2f} {s:5.2f} {d:+5.2f} {q['state'][:8]:8s} {g['status'][:22]:22s} pd={q['pd']['reason'][:12] if q.get('pd') else ''} gb={gbs[:30]:30s} {es[near(t,est)][1][:12]} ob={[(i,round(s_,2),round(d_,2),int(st)) for i,s_,d_,st,_ in o][:3]}")
