"""Reuses lap_sim; no ROS graph or vehicle. All gain experiments OFFLINE ONLY."""
import sys, json, argparse
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import compare_step39 as env
from controller.mpc.integrated_mpc import IntegratedMPC
from controller.combined.src.Controller import Controller
p=argparse.ArgumentParser(); p.add_argument('--variant', default='baseline'); p.add_argument('--tau',type=float,default=.12); p.add_argument('--pose-delay',type=int,default=0); p.add_argument('--real-deadlines',action='store_true'); a=p.parse_args()
variant=a.variant
orig=Controller._mpc_speed_command
samples=[]
def command(self,r,v):
    base=v if self.mpc_speed_cmd is None else self.mpc_speed_cmd
    out=orig(self,r,v)
    if r.success:
        samples.append([v,r.acceleration,r.speed,float(np.max(r.predicted[:,3])),base+r.acceleration*self.lateral_mpc.config.dt,out])
    if variant=='gain2' and r.success: return 2*out
    return out
Controller._mpc_speed_command=command
env.lap_sim.VESC_TAU=a.tau
env.lap_sim.REAL_DEADLINES=a.real_deadlines
ov=dict(mpc_speed_source='integrated',mpc_history_prediction=True,mpc_time_weighted_accel_cost=(variant=='time_cost'))
for name,vals in {'lead1':{'mpc_cmd_max_lead_mps':1.0},'nojerk':{'mpc_accel_jerk_mps3':0.},'q16':{'mpc_q_speed':16.},'rhalf':{'mpc_r_accel':.025},'poshalf':{'mpc_q_position':6.},'nopreview':{'mpc_steer_preview_s':0.},'nobrakelead':{'mpc_brake_lead_s':0.},'horizon16':{'mpc_horizon':16}}.items():
    if variant==name: ov.update(vals)
if variant!='pp':  # common 5 m/s / 4.5 m/s^2 reference; MPC ignores speed columns
    env.lap_sim.GB[:,2] = np.minimum(env.lap_sim.GB[:,2], np.minimum(5.,np.sqrt(4.5/np.maximum(abs(env.lap_sim.GB[:,5]),1e-6))))
r,t=env.lap_sim.simulate('pp' if variant.startswith('pp') else 'mpc','5p0',ov,laps=2,servo_delay_ticks=3,noise_pos=.02,seed=0,pose_delay_ticks=a.pose_delay)
r.pop('_status_seq');r.pop('binding_layer_pct',None)
k=np.interp(t['s'],env.lap_sim.FULL[:,4],env.lap_sim.FULL[:,5]); acc=np.diff(t['v'])/.02
r.update(variant=variant,tau=a.tau,pose_delay_ticks=a.pose_delay,real_deadlines=a.real_deadlines,map='test0',straight_accel_median=(float(np.median(acc[(abs(k[1:])<.1)&(acc>.1)])) if np.any((abs(k[1:])<.1)&(acc>.1)) else None),steer_rate_max=float(np.max(abs(np.diff(t['steer']))/.02)),accel_brake_reversals=int(np.sum((acc[1:]*acc[:-1]<0)&(abs(acc[1:]-acc[:-1])>.2))),speed_ceiling_violations=int(np.sum(t['cmd']>5.001)),ay_violations=int(np.sum(t['ay']>4.501)))
if samples:
    s=np.array(samples); r['solver_accel_p50_p95']=np.percentile(s[:,1],[50,95]).tolist();r['lead_bound_fraction']=float(np.mean(s[:,5]>=s[:,0]+ov.get('mpc_cmd_max_lead_mps',.5)-.001));r['predicted_peak_p95']=float(np.percentile(s[:,3],95))
np.savez_compressed(Path(__file__).parent/f'{variant}_{a.tau}_pose{a.pose_delay}{"_real" if a.real_deadlines else ""}.npz',**t,samples=np.array(samples))
print(json.dumps(r),flush=True)
