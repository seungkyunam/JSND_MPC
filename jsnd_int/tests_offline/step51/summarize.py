"""Postprocess saved trajectories only; does not rerun simulations."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import compare_step39 as env
D=Path(__file__).parent; full=env.lap_sim.FULL
cases=[('baseline','baseline_0.12_pose6.npz'),('time_cost','time_cost_0.12_pose6.npz'),('PP capped plan','pp_matched_0.12_pose6.npz')]
suffix='_real' if '--real' in sys.argv else ''
if suffix: cases=[(label,file.replace('.npz','_real.npz') if label!='PP capped plan' else file) for label,file in cases]
metrics=[];fig,axs=plt.subplots(3,1,figsize=(11,9),sharex=True)
for label,file in cases:
 t=np.load(D/file);s=t['s']; k=np.interp(s,full[:,4],full[:,5]); dk=np.interp(s,full[:,4],np.gradient(abs(full[:,5]),full[:,4])); curve=abs(k)>.12
 phases={'entry':curve&(dk>.05),'middle':curve&(abs(dk)<=.05),'exit':curve&(dk<-.05),'straight':~curve}
 acc=np.diff(np.r_[0.,t['v']])/.02;sr=np.diff(np.r_[0.,t['steer']])/.02
 coupled=(acc/5.)**2+(t['ay']/4.5)**2
 row={'controller':label,'phases':{},'coupled_plant_violations':int(np.sum(coupled>1.001)),'coupled_plant_max':float(coupled.max()),'steer_variation_rad':float(np.sum(abs(np.diff(t['steer'])))),'steer_oscillations':int(np.sum((sr[1:]*sr[:-1]<0)&(abs(sr[1:])>.1)&(abs(sr[:-1])>.1))),'accel_max':float(acc.max()),'decel_min':float(acc.min()),'command_accel_max':float(np.max(np.diff(t['cmd'])/.02)),'command_decel_min':float(np.min(np.diff(t['cmd'])/.02))}
 for phase,mask in phases.items():
  row['phases'][phase]={'samples':int(mask.sum()),'speed_mean':float(t['v'][mask].mean()),'lateral_abs_mean':float(abs(t['d'][mask]).mean()),'lateral_abs_max':float(abs(t['d'][mask]).max()),'inside_error_max':float(np.max(t['d'][mask]*np.sign(k[mask])))}
 bins=np.linspace(0,full[-1,4],43);centers=(bins[1:]+bins[:-1])/2
 for ax,key in zip(axs,['v','d','steer']):
  vals=[float(np.mean(t[key][(s>=lo)&(s<hi)])) if np.any((s>=lo)&(s<hi)) else np.nan for lo,hi in zip(bins,bins[1:])]
  ax.plot(centers,vals,label=label);ax.grid(alpha=.2)
 metrics.append(row)
for ax,y in zip(axs,['Speed (m/s)','Signed cross-track (m)','Steering (rad)']):ax.set_ylabel(y)
axs[0].legend();axs[-1].set_xlabel('test0 path position (m)');fig.suptitle('Offline only: pose delay 120 ms, servo delay 60 ms, VESC tau 0.12 s')
fig.tight_layout();fig.savefig(D/f'profiles{suffix}.png',dpi=140)
(D/f'phase_metrics{suffix}.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics,indent=2))
