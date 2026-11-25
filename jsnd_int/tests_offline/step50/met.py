import re,sys,collections,statistics as st
def run(f):
    ev=[]
    for line in open(f,errors='ignore'):
        m=re.search(r'\[(\d{10}\.\d+)\] \[([^\]]+)\]: (.*)',line)
        if m: ev.append((float(m.group(1)),m.group(3)))
    t0=[t for t,m in ev if 'autodrive engaged' in m][0]
    ends=[t for t,m in ev if 'manual drive' in m and t>t0]
    t1=ends[0] if ends else ev[-1][0]
    E=[(t-t0,m) for t,m in ev if t0<=t<=t1]
    T=t1-t0
    stops=collections.Counter(m.split('STOP requested: ')[1].split(':')[0] if not m.split('STOP requested: ')[1].startswith(('envelope_recovery','nonlinear','invalid')) else ':'.join(m.split('STOP requested: ')[1].split(':')[:2]) for t,m in E if 'STOP requested' in m)
    nst=sum(stops.values())
    st_t=[t for t,m in E if 'STOP requested' in m]
    epis=sum(1 for i,t in enumerate(st_t) if i==0 or t-st_t[i-1]>0.5)
    brk=[(t,float(re.search(r'target=([\d.]+)',m).group(1)),float(re.search(r'actual=([\d.]+)',m).group(1))) for t,m in E if 'braking engaged' in m]
    ttc=[t for t,m in E if 'TTC' in m]
    sm=[(t,float(re.search(r'speed=(-?[\d.]+)',m).group(1)),re.search(r'state=(\w+)',m).group(1)) for t,m in E if 'STATIC_OT]' in m]
    laps=[float(m.split(' in ')[1]) for t,m in E if 'completed lap' in m]
    # standstill time from SM samples (<0.15) approx: count consecutive samples
    stand=0; prev=None
    for t,v,s in sm:
        if prev and prev[1]<0.15 and v<0.15: stand+=t-prev[0]
        prev=(t,v,s)
    # O->T transition speeds
    ot=[v for (a,b) in zip(sm,sm[1:]) for v in [b[1]] if a[2]=='OVERTAKE' and b[2]=='TRAILING' and b[1]>1.0]
    rec=[t for t,m in E if 'envelope_recovery' in m]
    allv=[v for _,v,_ in sm]+[a for _,_,a in brk]
    print(f'== {f}  autodrive {T:.1f}s')
    print(f' laps {len(laps)}: {laps}')
    if laps: print(f'  best {min(laps):.2f} median {st.median(laps):.2f}')
    print(f' STOP {nst} ({nst/T*60:.1f}/min), episodes(>0.5s gap) {epis} ({epis/T*60:.1f}/min)')
    for k,v in stops.most_common(): print(f'   {v:3d} {k}')
    z=[b for b in brk if b[1]<0.05]
    print(f' VESC braking engaged {len(brk)} ({len(brk)/T*60:.1f}/min), target=0 {len(z)} ({len(z)/T*60:.1f}/min), median actual at target0 {st.median([b[2] for b in z]) if z else 0:.2f}')
    nz=[b[2]-b[1] for b in brk if b[1]>=0.05]
    if nz: print(f'  nonzero brake events: median overshoot actual-target {st.median(nz):.2f}')
    print(f' TTC {len(ttc)} ({len(ttc)/T*60:.1f}/min)')
    print(f' standstill (SM v<0.15) ~{stand:.1f}s = {stand/T*100:.0f}%')
    print(f' OVERTAKE->TRAILING at v>1: n={len(ot)} speeds {ot}')
    print(f' envelope_recovery events {len(rec)}; max speed seen {max(allv):.2f}')
for f in sys.argv[1:]: run(f)
