#!/usr/bin/env python3
"""Step 42: offline-only curvature margin ablation with baseline diagnostics.

OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/compare_step42.py --factor 0.85
OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/compare_step42.py --factor 1.0

Uses the existing test0 lap simulator and real deadlines; each invocation runs
one case, two laps. Changing the class constant is process-local: no runtime
configuration or real-car controller source is edited. Diagnostics run outside
solver elapsed timing. Binding/near-cap fractions are descriptive, not causal
attributions. Near-cap uses v >= cap - 0.15 m/s, including overshoots; command
fractions cover successful solves only. The opposing-direction metric requires
acceleration * (command - measured speed) < -0.01.
"""
import sys,json,argparse
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import compare_step39 as env
from controller.mpc.integrated_mpc import IntegratedMPC,_gradient
from controller.combined.src.Controller import Controller
parser=argparse.ArgumentParser()
parser.add_argument('--factor',type=float,choices=(.85,1.0),default=.85)
args=parser.parse_args()
IntegratedMPC.LATERAL_PLAN_FRACTION=args.factor
rows=[];commands=[]
oldsolve=IntegratedMPC.solve
oldcmd=Controller._mpc_speed_command

def solve(self,state,path,*args,**kw):
    r=oldsolve(self,state,path,*args,**kw)
    arc,caps=self._geometry_caps(path,np.inf)
    tang=_gradient(path[:,:2],arc); k=abs(_gradient(np.unwrap(np.arctan2(tang[:,1],tang[:,0])),arc))
    seg=np.diff(path[:,:2],axis=0); f=np.clip(np.sum((state[:2]-path[:-1,:2])*seg,axis=1)/np.sum(seg*seg,axis=1),0,1)
    j=np.argmin(np.sum((path[:-1,:2]+f[:,None]*seg-state[:2])**2,axis=1)); s=arc[j]+f[j]*(arc[j+1]-arc[j])
    local=min(self.config.max_speed,np.sqrt(self.LATERAL_PLAN_FRACTION*self.config.max_lateral_accel/max(np.interp(s,arc,k),1e-6)))
    cap=np.interp(s,arc,caps)
    rows.append([state[3],local,cap])
    return r

def cmd(self,r,measured):
    out=oldcmd(self,r,measured)
    if r.success:
        commands.append([r.acceleration,out-measured,out>=measured+.5-1e-3,out>=max(r.predicted[:,3])-1e-3])
    return out
IntegratedMPC.solve=solve;Controller._mpc_speed_command=cmd
env.lap_sim.REAL_DEADLINES=True
r,t=env.lap_sim.simulate('mpc','8p88',{'mpc_speed_source':'integrated','mpc_max_lateral_accel_mps2':8.,'mpc_max_accel_mps2':6.,'mpc_max_decel_mps2':6.},rate=50,laps=2)
r.pop('_status_seq');r.pop('binding_layer_pct')
a=np.array(rows);c=np.array(commands)
r['diagnostics']={'lead_binding_fraction':float(c[:,2].mean()),'horizon_binding_fraction':float(c[:,3].mean()),'accel_command_opposes_plan_fraction':float(np.mean(c[:,0]*c[:,1]<-.01)),'near_curvature_cap_fraction':float(np.mean(a[:,0]>=a[:,1]-.15)),'backward_below_local_fraction':float(np.mean(a[:,2]<a[:,1]-.15)),'near_backward_cap_fraction':float(np.mean(a[:,0]>=a[:,2]-.15))}
r.update(step=42,plan_fraction=args.factor,map='test0',rate_hz=50,
         speed_ceiling_mps=8.88,lateral_limit_mps2=8.,accel_limit_mps2=6.,decel_limit_mps2=6.,
         plant_accel_mps2=env.lap_sim.VESC_ACC,plant_decel_mps2=env.lap_sim.VESC_DEC,plant_tau_s=env.lap_sim.VESC_TAU)
print(json.dumps(r))
