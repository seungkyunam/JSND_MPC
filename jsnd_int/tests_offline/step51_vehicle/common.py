import pickle,numpy as np,collections,re
D=pickle.load(open('d.pkl','rb'))
L='/home/jsnd/bench_logs/step45_mpc_5p0_20261126_181959.log'
txt=open(L).read()
ends=[float(x) for x in re.findall(r'\[(\d+\.\d+)\] \[lap_analyser\]: LapAnalyser: completed lap',txt)]
start=float(re.search(r'\[(\d+\.\d+)\] \[lap_analyser\]: LapAnalyser: started first lap',txt).group(1))
B=[start]+ends
ttc=[float(x) for x in re.findall(r'\[(\d+\.\d+)\] \[simple_mux\]: EStop: TTC',txt)]
def arr(t,cols=None):
    v=D[t]; return np.array([x[0] for x in v]), v
def lap_of(ts):
    for i in range(len(B)-1):
        if B[i]<=ts<B[i+1]: return i+1
    return 0
fr=np.array([x for x in D['/car_state/odom_frenet']])
def s_at(ts):
    i=np.searchsorted(fr[:,0],ts); i=min(i,len(fr)-1); return fr[i,1],fr[i,2],fr[i,3]
