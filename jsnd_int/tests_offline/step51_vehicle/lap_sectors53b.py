import pickle,numpy as np,collections,bisect
D=pickle.load(open('D.pkl','rb'))
F=np.array(D['/car_state/odom_frenet']); t,s,dd,v=F.T; L=s.max()
wr=np.where(np.diff(s)<-L/2)[0]+1; laps=[(wr[i],wr[i+1]) for i in range(len(wr)-1)]
lt=np.array([t[b]-t[a] for a,b in laps]); print('mean %.3f std %.3f'%(lt.mean(),lt.std(ddof=1)))
ft=t
def sat(tt): i=min(len(ft)-1,bisect.bisect_left(ft,tt)); return s[i]
# zero high-level commands location & status
H=D['/vesc/high_level/ackermann_cmd']; ST=D['/controller/mpc_status']; stt=[x[0] for x in ST]
SM=D['/state_machine']; smt=[x[0] for x in SM]
z=collections.Counter()
for tt,sp,_ in H:
    if t[laps[0][0]]<=tt<t[laps[-1][1]] and sp<0.05:
        k=ST[max(0,bisect.bisect_right(stt,tt)-1)][1].split(':')[0]; z[(round(sat(tt)),k,SM[max(0,bisect.bisect_right(smt,tt)-1)][1])]+=1
print('zero hl cmd by (s,status,state):',z.most_common(12))
# time per 1 m bin
bins=np.arange(0,14.5,1.0)
def tb(a,b):
    out=[]
    for i in range(len(bins)-1):
        m=(s[a:b]>=bins[i])&(s[a:b]<bins[i+1]); out.append(m.sum()*np.median(np.diff(t[a:b])))
    return np.array(out)
TB=np.array([tb(a,b) for a,b in laps]); o=np.argsort(lt)
fast,slow=TB[o[:4]].mean(0),TB[o[-4:]].mean(0)
print('bin  fast  slow  diff  std_all')
for i in range(len(fast)): print(f'{bins[i]:4.0f} {fast[i]:.2f} {slow[i]:.2f} {slow[i]-fast[i]:+.2f} {TB[:,i].std():.2f}')
