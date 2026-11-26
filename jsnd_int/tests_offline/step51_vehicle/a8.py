from common import *
import pickle
G=np.array(pickle.load(open('gw.pkl','rb')))
dg=D['/controller/mpc_diagnostics']; hl=np.array(D['/vesc/high_level/ackermann_cmd'])
pf=np.array(D['/pf/pose/odom'])
n=lambda t,ts:min(np.searchsorted(ts,t),len(ts)-1)
R=collections.defaultdict(list)
for ts,j in dg:
    if not B[0]<=ts<B[-1]: continue
    st=re.sub(r'_late|_inexact_verified','',j['status'])
    if st.startswith('STOP') : continue
    p=pf[n(ts,pf[:,0])]; k=np.argmin(np.hypot(G[:,1]-p[1],G[:,2]-p[2]))
    he=np.angle(np.exp(1j*(p[3]-G[k,3])))
    v=j.get('measured_mps') or 0
    if v<1.0: continue
    R[st].append((abs(j.get('steering_rad') or 0), abs(hl[n(ts,hl[:,0]),2]), G[k,4], he, j.get('steering_rad') or 0))
for k,v in sorted(R.items(),key=lambda x:-len(x[1])):
    a=np.array(v)
    # steering vs needed: sign agreement with heading error correction (+he => steer negative)
    print(f"{k[:45]:45s} n={len(a):5d} |steer|med={np.median(a[:,0]):.3f} p90={np.percentile(a[:,0],90):.3f} frac|steer|<0.02={np.mean(a[:,0]<0.02):.2f} kappa_med={np.median(a[:,2]):.2f} |hdgErr|med={np.median(np.abs(a[:,3])):.2f}")
