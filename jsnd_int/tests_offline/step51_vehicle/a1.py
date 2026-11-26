import pickle,numpy as np
D=pickle.load(open('d.pkl','rb'))
for t,v in D.items():
    ts=np.array([x[0] for x in v]); g=np.diff(ts)
    print(f"{t:40s} n={len(v)} hz={len(v)/(ts[-1]-ts[0]):.1f} maxgap={g.max():.3f} gaps>0.2={int((g>0.2).sum())}")
