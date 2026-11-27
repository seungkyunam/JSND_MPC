import json, csv, sys, hashlib, subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]; OUT=Path(__file__).parent/'results'
plt.rcParams.update({'figure.dpi':140,'font.size':11,'axes.grid':True})
def savefig(fig,path):
    fig.text(.5,.008,'Model study only: tire saturation and vehicle-body collisions are not simulated.',ha='center',fontsize=9,color='#9b2226')
    fig.tight_layout(rect=(0,.025,1,1));fig.savefig(path);plt.close(fig)
def writecsv(name, rows):
    if not rows:return
    with (OUT/name).open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def episodes(mask, minimum=1):
    edges=np.diff(np.r_[False,mask,False].astype(int))
    return int(np.sum(np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)>=minimum))
def mean(x):return float(np.mean(x)) if len(x) else None
summaries=[]; phases=[]; segments=[]; laps=[]; data={}
for file in sorted(OUT.glob('*_run.json')):
    run=json.loads(file.read_text()); name=run['name']; delay=run['pose_delay_ticks']
    full=np.genfromtxt(OUT/f'{name}_trace.csv',delimiter=',',names=True)
    t=full[full['lap']>1]; data[(delay,run['controller'])]=(run,t)
    lt=np.array(run['lap_times'][1:]); n=len(lt)
    for i,l in enumerate(run['lap_times']):laps.append(dict(run=name,lap=i+1,time_s=l,in_statistics=i>0))
    ds=np.diff(t['delta'])/.02; cs=np.diff(t['steer'])/.02
    straight=np.abs(t['kappa'])<.12
    positive=straight&(t['a']>.1)
    diag=[json.loads(s) for s in (OUT/f'{name}_diagnostics.jsonl').read_text().splitlines()]
    diag=[r for r,keep in zip(diag,full['lap']>1) if keep]
    row=dict(run=name,controller=run['controller'],pose_delay_ms=delay*20,completed_laps=run['completed_laps'],stat_laps=n,
      best_lap_s=float(lt.min()),mean_lap_s=float(lt.mean()),lap_std_sample_s=float(lt.std(ddof=1)),
      max_speed_mps=float(t['v'].max()),mean_speed_mps=mean(t['v']),
      lateral_mae_m=mean(abs(t['d'])),lateral_rmse_m=float(np.sqrt(mean(t['d']**2))),
      lateral_p95_m=float(np.percentile(abs(t['d']),95)),lateral_max_m=float(abs(t['d']).max()),
      actual_steer_rms_rad=float(np.sqrt(mean(t['delta']**2))),actual_steer_max_rad=float(abs(t['delta']).max()),
      actual_steer_rate_rms_radps=float(np.sqrt(mean(ds**2))),actual_steer_rate_max_radps=float(abs(ds).max()),
      command_steer_max_rad=float(abs(t['steer']).max()),command_rate_max_radps=float(abs(cs).max()),
      straight_accel_mean_mps2=mean(t['a'][straight]),straight_positive_accel_mean_mps2=mean(t['a'][positive]),
      straight_positive_accel_p95_mps2=float(np.percentile(t['a'][positive],95)) if positive.any() else None,
      actual_stops=episodes(t['v']<.1,10),solver_failure_events=episodes(t['solver_failure']>0),
      collision_count=None,rear_axle_occupied_events=episodes(t['point_occupied']>0),
      motor_cap_pct=100*mean(t['cmd']>t['motor_cmd']+1e-6),
      actual_ay_max_mps2=float(t['ay'].max()),actual_ay_over_4p5_pct=100*mean(t['ay']>4.5),
      combined_ellipse_over_pct=100*mean((t['a']/np.where(t['a']>=0,4.,6.))**2+(t['ay']/4.5)**2>1.),
      combined_ellipse_max=float(np.max((t['a']/np.where(t['a']>=0,4.,6.))**2+(t['ay']/4.5)**2)),
      lead_limited_pct=100*mean(np.array([r.get('lead_limited',False) for r in diag])),
      plan_limited_pct=100*mean(np.array([r.get('plan_limited',False) for r in diag])),
      jerk_limited_pct=100*mean(np.array([r.get('jerk_limited',False) for r in diag])),
      solver_failure_pct=100*mean(t['solver_failure']),
      lead_1p2_pct=100*mean(np.array([r.get('lead_mps',0)>=1.199 for r in diag])))
    summaries.append(row)
    # Derivative on fixed path geometry, avoiding speed-dependent phase definitions.
    track=json.loads((ROOT/'jsnd_int/racing_stack/stack_master/maps/11_18_6/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    arc=np.array([w['s_m'] for w in track]); k=np.abs([w['kappa_radpm'] for w in track])
    slope=np.interp(t['s'],arc,np.gradient(k,arc))
    labels=np.where(straight,'straight',np.where(slope>.05,'entry',np.where(slope<-.05,'exit','middle')))
    for phase in ['straight','entry','middle','exit']:
        m=labels==phase
        phases.append(dict(run=name,phase=phase,time_per_lap_s=m.sum()*.02/n,mean_speed_mps=mean(t['v'][m]),
          mean_accel_mps2=mean(t['a'][m]),lateral_rmse_m=float(np.sqrt(mean(t['d'][m]**2))) if m.any() else None,
          lead_limited_pct=100*mean(np.array([r.get('lead_limited',False) for r in diag])[m]),
          jerk_limited_pct=100*mean(np.array([r.get('jerk_limited',False) for r in diag])[m])))
    for b in range(int(np.ceil(arc[-1]))):
        m=(t['s']>=b)&(t['s']<b+1)
        segments.append(dict(run=name,s_start_m=b,s_end_m=min(b+1,arc[-1]),time_per_lap_s=m.sum()*.02/n,
          mean_speed_mps=mean(t['v'][m]),mean_accel_mps2=mean(t['a'][m]),lateral_mae_m=mean(abs(t['d'][m])),
          mean_plan_mps=mean(t['plan_v'][m]),mean_abs_curvature=mean(abs(t['kappa'][m]))))
writecsv('summary.csv',summaries);writecsv('laps.csv',laps);writecsv('phases.csv',phases);writecsv('segments.csv',segments)
paired=[]
for delay in sorted(set(k[0] for k in data)):
    fig,axs=plt.subplots(2,2,figsize=(13,8))
    for ctrl,col in [('pp','#1f77b4'),('mpc','#d95f02')]:
        run,t=data[delay,ctrl]; label=ctrl.upper()
        axs[0,0].plot(range(2,12),run['lap_times'][1:],'o-',label=label,color=col)
        sg=[r for r in segments if r['run']==run['name']]; s=[r['s_start_m']+.5 for r in sg]
        axs[0,1].plot(s,[r['mean_speed_mps'] for r in sg],'o-',label=label,color=col)
        axs[1,0].plot(s,[r['lateral_mae_m'] for r in sg],'o-',label=label,color=col)
        axs[1,1].plot(s,[r['time_per_lap_s'] for r in sg],'o-',label=label,color=col)
    for ax,title,x,y in zip(axs.flat,['Flying lap times','Speed by path segment','Tracking error','Segment traversal time'],
       ['Lap','Path s (m)','Path s (m)','Path s (m)'],['Time (s)','Speed (m/s)','MAE (m)','Time per lap (s)']):
        ax.set(title=title,xlabel=x,ylabel=y);ax.legend()
    fig.suptitle(f'11_18_6 | OFFLINE kinematic model | pose delay {delay*20} ms | 10 flying laps')
    savefig(fig,OUT/f'comparison_delay{delay}.png')
    pp=[r for r in segments if r['run']==data[delay,'pp'][0]['name']]
    mp=[r for r in segments if r['run']==data[delay,'mpc'][0]['name']]
    for p,m in zip(pp,mp):paired.append(dict(pose_delay_ms=delay*20,s_start_m=p['s_start_m'],
       pp_time_s=p['time_per_lap_s'],mpc_time_s=m['time_per_lap_s'],mpc_minus_pp_s=m['time_per_lap_s']-p['time_per_lap_s'],
       pp_speed_mps=p['mean_speed_mps'],mpc_speed_mps=m['mean_speed_mps']))
writecsv('segment_differences.csv',paired)
# Additional geometric corner thirds: >= .35 /m contiguous regions, divided
# into three equal arc-length parts; chosen from geometry, identical for both.
corner_rows=[]; runs_mask=k>=.35
starts=np.flatnonzero(np.diff(np.r_[False,runs_mask].astype(int))==1)
ends=np.flatnonzero(np.diff(np.r_[runs_mask,False].astype(int))==-1)
for (delay,ctrl),(run,t) in sorted(data.items()):
    for corner,(a,b) in enumerate(zip(starts,ends),1):
        bounds=np.linspace(arc[a],arc[b],4)
        for i,phase in enumerate(['entry','middle','exit']):
            m=(t['s']>=bounds[i])&(t['s']<bounds[i+1])
            corner_rows.append(dict(run=run['name'],corner=corner,phase=phase,s_start_m=bounds[i],s_end_m=bounds[i+1],
                mean_speed_mps=mean(t['v'][m]),mean_accel_mps2=mean(t['a'][m]),time_per_lap_s=m.sum()*.02/10))
writecsv('corner_thirds.csv',corner_rows)
fig,axs=plt.subplots(1,2,figsize=(12,5))
for corner,ax in enumerate(axs,1):
    for ctrl,col in [('pp','#1f77b4'),('mpc','#d95f02')]:
        rows=[r for r in corner_rows if r['run']==data[0,ctrl][0]['name'] and r['corner']==corner]
        ax.plot([r['phase'] for r in rows],[r['mean_speed_mps'] for r in rows],'o-',label=ctrl.upper(),color=col)
    ax.set(title=f'Corner {corner}',ylabel='Mean speed (m/s)');ax.legend()
fig.suptitle('Geometric corner thirds | OFFLINE | 0 ms pose delay');savefig(fig,OUT/'corner_speeds.png')
fig,axs=plt.subplots(3,1,figsize=(13,10))
for ctrl,col in [('pp','#1f77b4'),('mpc','#d95f02')]:
    run,t=data[0,ctrl]; q=t[t['lap']==11]
    axs[0].plot(q['s'],q['v'],label=ctrl.upper(),color=col)
    axs[1].plot(q['s'],q['delta'],label=ctrl.upper(),color=col)
    axs[2].plot(q['s'][1:],np.diff(q['delta'])/.02,label=ctrl.upper(),color=col)
for ax,y in zip(axs,['Speed (m/s)','Actual steering (rad)','Actual steering rate (rad/s)']):
    ax.set(xlabel='Path s (m)',ylabel=y);ax.legend()
fig.suptitle('Last flying lap | OFFLINE | 0 ms pose delay');savefig(fig,OUT/'steering_speed.png')
fig,axs=plt.subplots(1,2,figsize=(12,5))
for ctrl,col in [('pp','#1f77b4'),('mpc','#d95f02')]:
    rows=sorted([r for r in summaries if r['controller']==ctrl],key=lambda r:r['pose_delay_ms'])
    for ax,key in zip(axs,['mean_lap_s','lateral_rmse_m']):
        ax.plot([r['pose_delay_ms'] for r in rows],[r[key] for r in rows],'o-',label=ctrl.upper(),color=col)
for ax,y in zip(axs,['Mean lap (s)','Lateral RMSE (m)']):ax.set(xlabel='Shared pose delay (ms)',ylabel=y);ax.legend()
fig.suptitle('Delay sensitivity | OFFLINE, uncalibrated plant');savefig(fig,OUT/'delay_sensitivity.png')
fig,axs=plt.subplots(1,2,figsize=(12,5))
for ctrl,col in [('pp','#1f77b4'),('mpc','#d95f02')]:
    r,t=data[0,ctrl]
    q=(t['a']/np.where(t['a']>=0,4.,6.))**2+(t['ay']/4.5)**2
    last=t['lap']==11
    axs[0].plot(t['s'][last],q[last],label=ctrl.upper(),color=col)
    axs[1].bar(ctrl.upper(),100*np.mean(q>1),color=col)
axs[0].axhline(1,color='black',linestyle='--',label='Assumed ellipse limit')
axs[0].set(xlabel='Path s (m)',ylabel='(a / A)^2 + (ay / 4.5)^2');axs[0].legend()
axs[1].set(ylabel='Flying-lap ticks above assumed limit (%)')
fig.suptitle('Physical feasibility audit | 0 ms pose delay | BOTH fail combined-limit check')
savefig(fig,OUT/'physical_limit_audit.png')
manifest={'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
 'python':sys.version,'files':{}}
for base in ['jsnd_int/racing_stack/controller','jsnd_int/racing_stack/stack_master/config','stack_master/config',
 'jsnd_int/racing_stack/stack_master/maps/11_18_6','jsnd_int/tests_offline/lap_sim.py','benchmark_final/run.py','benchmark_final/analyze.py','benchmark_final/report.py']:
    path=ROOT/base
    for p in path.rglob('*') if path.is_dir() else [path]:
        if p.is_file() and '__pycache__' not in str(p):manifest['files'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(summaries,indent=2))
