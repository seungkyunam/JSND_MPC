"""Identical-state PP/MPC output probes and isolated straight acceleration tests."""
import sys,json,csv
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import compare_step39 as env
L=env.lap_sim
L.GB[:,2]=np.minimum(L.GB[:,2],np.minimum(5.,np.sqrt(4.5/np.maximum(abs(L.GB[:,5]),1e-6))))
OUT=Path(__file__).parent
rows=[]
for i in range(0,len(L.GB),20):
 p=L.GB[i]
 for v in (1.,2.,3.):
  for variant in ('baseline','time_cost','pp'):
   c,_=L.build('pp' if variant=='pp' else 'mpc','5p0',{'mpc_speed_source':'integrated','mpc_history_prediction':True,'mpc_time_weighted_accel_cost':variant=='time_cost'},50)
   steer=float(np.clip(np.arctan(.324*p[5]),-.4,.4));c.curr_steering_angle=steer;c.current_steer_command=steer;c.curr_applied_steering=steer;c.mpc_speed_cmd=v
   c.pose_prediction.clock=lambda:1.
   for t in np.arange(.7,1.,.02):c.pose_prediction.commands.append((float(t),steer))
   c.mpc_applied_accel=0.
   out=c.main_loop('GB_TRACK',np.array([[p[0],p[1],p[6]]]),L.local_wpnts(p[4]),v,None,np.array([p[4],0.,v,0.]),np.zeros(10),L.TRACK_LEN)
   row=dict(i=i,s=float(p[4]),measured=v,variant=variant,command=float(out[0]),steer=float(out[3]))
   if variant!='pp':row.update(c.mpc_speed_diagnostics)
   rows.append(row)
(OUT/'same_state.json').write_text(json.dumps(rows,indent=2))
if '--same-state-only' in sys.argv: raise SystemExit(0)
# Standalone straight: 20m valid horizon, speed feedback and VESC response only.
summary=[]
for tau in (.12,.45):
 for variant in ('baseline','time_cost','lead1','nojerk','gain2'):
  c,_=L.build('mpc','5p0',dict(mpc_speed_source='integrated',mpc_time_weighted_accel_cost=variant=='time_cost',mpc_cmd_max_lead_mps=1. if variant=='lead1' else .5,mpc_accel_jerk_mps3=0. if variant=='nojerk' else 20.),50)
  v=0.; trace=[]
  path=np.column_stack((np.linspace(0,20,201),np.zeros(201),np.full(201,5.)))
  for j in range(250):
   r=c.lateral_mpc.solve(np.array([0.,0.,0.,v]),path,0.)
   if not r.success:
    if variant=='gain2': break
    raise RuntimeError(r.status)
   cmd=c._mpc_speed_command(r,v)*(2. if variant=='gain2' else 1.)
   acc=float(np.clip((cmd-v)/tau,-6.,4.));trace.append([j*.02,v,r.acceleration,r.speed,np.max(r.predicted[:,3]),c.mpc_speed_diagnostics['command_unclipped_mps'],cmd,acc,int(c.mpc_speed_diagnostics['lead_limited'])]);v=max(0.,v+.02*acc)
  t=np.array(trace);mask=(t[:,1]>=.5)&(t[:,1]<3.)
  summary.append(dict(variant=variant,tau=tau,terminal_status=r.status,elapsed_s=j*.02,accel_0p5_to_3_median=float(np.median(t[mask,7])),time_to_3=float(t[t[:,1]>=3.,0][0]) if np.any(t[:,1]>=3.) else None,lead_limited_fraction=float(t[:,8].mean()),max_speed=float(t[:,1].max()),max_command=float(t[:,6].max()),optimal_accel_median=float(np.median(t[mask,2]))))
  np.savetxt(OUT/f'straight_{variant}_{tau}.csv',t,delimiter=',',header='t,v,optimal_a,predicted_first,predicted_peak,preclip,command,actual_a,lead_limited',comments='')
(OUT/'straight_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
