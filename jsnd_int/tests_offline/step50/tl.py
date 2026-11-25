import re,sys,collections
f=sys.argv[1]
ts=re.compile(r'\[(\d{10}\.\d+)\] \[([^\]]+)\]: (.*)')
ev=[]
for line in open(f,errors='ignore'):
    m=ts.search(line)
    if not m: continue
    t=float(m.group(1)); n=m.group(2); msg=m.group(3)
    ev.append((t,n,msg))
eng=[e for e in ev if 'autodrive engaged' in e[2]]
t0=eng[0][0] if eng else ev[0][0]
def K(msg):
    if 'STOP requested' in msg: return 'STOP '+msg.split('STOP requested: ')[1]
    if 'Current braking engaged' in msg: return 'BRK '+msg.split(': ',1)[1]
    if 'Current braking released' in msg: return 'REL '+msg.split(': ',1)[1]
    if 'TTC' in msg: return 'TTC'
    if 'completed lap' in msg or 'started first lap' in msg: return 'LAP '+msg.split(': ')[-1]
    if '[STATIC_AVOID] obs_id=-' in msg: return None
    if 'STATIC_AVOID' in msg: return 'OBS '+msg.split('] ',1)[1]
    if 'STATIC_OT]' in msg:
        st=re.search(r'state=(\w+)',msg).group(1); sp=re.search(r'speed=(-?[\d.]+)',msg).group(1)
        return f'SM {st} v={sp}'
    if '[safety]' in msg: return 'SAFE '+msg.split('[safety] ')[1][:50]
    if '[MPC]' in msg and 'tracking local' not in msg: return 'MPC '+msg.split('[MPC] ')[1]
    return None
for t,n,msg in ev:
    k=K(msg)
    if k and t>=t0-1: print(f'{t-t0:8.3f} {k}')
