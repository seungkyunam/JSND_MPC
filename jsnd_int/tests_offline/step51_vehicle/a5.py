from common import *
import sys
lap=int(sys.argv[1]); t0,t1=map(float,sys.argv[2:4]); a=B[lap-1]
sd=D['/state_machine/debug']; ob=D['/tracking/obstacles']; obt=np.array([x[0] for x in ob])
last=None
for ts,q in sd:
    if not a+t0<=ts<a+t1: continue
    o=ob[min(np.searchsorted(obt,ts),len(ob)-1)][1]
    k=(q['state'],str(q['gb']),str(q['pd']))
    if k==last: continue
    last=k
    print(f"{ts-a:5.2f} {q['state']:9s} s={q['cur_s']:.2f} d={q['cur_d']:+.2f} gb={[ (x['id'],x['gap'],x['d'],x['blocked'],x['branch']) for x in (q['gb'] or {}).get('obs',[])]} pd={q['pd']} ob={[(i,round(s_,2),round(d_,2),int(st)) for i,s_,d_,st,_ in o]}")
