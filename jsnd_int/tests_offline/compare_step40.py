#!/usr/bin/env python3
"""Step 40: offline lateral-limit comparison, with real-car defaults unchanged.

Run from the repository with numpy/scipy/osqp/pyyaml installed:
  OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/compare_step40.py

Archived results were produced before this archival commit, using the same
script with an absolute import path. No experiment was rerun for archival.
First-stage diagnostics cover successful plans only, not measured tire forces.
The 5 m/s^2 longitudinal limit is fixed in all three cases.
"""
import sys,json
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import compare_step39 as env
from controller.mpc.integrated_mpc import IntegratedMPC
lap=env.lap_sim
lap.REAL_DEADLINES=True
original=IntegratedMPC.solve
rows=[]
def solve(self,*args,**kwargs):
    r=original(self,*args,**kwargs)
    if r.success:
        c=self.config
        a,d=r.controls[0]
        v=max(r.predicted[0,3],r.predicted[1,3])
        ay=v*v*abs(np.tan(d))/c.wheelbase
        utilization=(a/(c.max_accel if a>=0 else c.max_decel))**2+(ay/c.max_lateral_accel)**2
        available=5*np.sqrt(max(0,1-(ay/c.max_lateral_accel)**2))
        rows.append([a,ay,utilization,available])
    return r
IntegratedMPC.solve=solve
for limit in [4.5,6.,8.]:
    rows.clear()
    r,t=lap.simulate('mpc','8p88',{'mpc_speed_source':'integrated','mpc_max_lateral_accel_mps2':limit},rate=50,laps=2)
    r.pop('_status_seq');r.pop('binding_layer_pct')
    a=np.asarray(rows)
    r.update(lateral_limit=limit,first_stage_ellipse_over_90pct=float(np.mean(a[:,2]>.9)),first_stage_ay_mean=float(a[:,1].mean()),available_longitudinal_p10_median=np.percentile(a[:,3],[10,50]).tolist())
    print(json.dumps(r),flush=True)
