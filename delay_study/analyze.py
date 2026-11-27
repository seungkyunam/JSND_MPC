"""Analyze saved traces, including failed completion, without rerunning controllers."""
import csv,json,hashlib,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=Path(__file__).parent/'results'
plt.rcParams.update({'font.size':10,'figure.dpi':160,'axes.grid':True})
def csvout(name,rows):
    if not rows:return
    with (OUT/name).open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
def avg(a):return float(np.mean(a)) if len(a) else float('nan')
def rmse(a):return float(np.sqrt(np.mean(a*a))) if len(a) else None
def events(mask,n):
    e=np.diff(np.r_[False,mask,False].astype(int));return int(np.sum(np.flatnonzero(e==-1)-np.flatnonzero(e==1)>=n))
def stops(t,n):
    started=np.maximum.accumulate(t['v']>.3)
    return events(started&(t['v']<.1),n)
def save(fig,name):
    fig.text(.5,.008,'OFFLINE uncalibrated model: no tire saturation/contact physics. Real logs are NOT matched trials.',ha='center',fontsize=8,color='darkred')
    fig.tight_layout(rect=(0,.025,1,.96));fig.savefig(OUT/name);plt.close(fig)
wp=json.loads((ROOT/'jsnd_int/racing_stack/stack_master/maps/11_18_6/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
s=np.array([p['s_m'] for p in wp]);k=np.abs([p['kappa_radpm'] for p in wp]);mask=k>=.35
starts=np.flatnonzero(np.diff(np.r_[False,mask].astype(int))==1);ends=np.flatnonzero(np.diff(np.r_[mask,False].astype(int))==-1)
summary=[];corners=[];laprows=[];sources=[]
for path in sorted(OUT.glob('*_run.json')):
    r=json.loads(path.read_text());name=r['name'];ph=r['physical']
    a=np.genfromtxt(OUT/f'{name}_trace.csv',names=True,delimiter=',')
    m=(a['lap']>1)&(a['lap']<=r['completed_laps']);t=a[m]
    lt=np.array(r['lap_times'][1:]);ms=r['pose_delay_ms'];mode='PP' if r['controller']=='pp' else ('MPC ON' if r['compensation_s'] else 'MPC OFF')
    d=[json.loads(l) for l in (OUT/f'{name}_diagnostics.jsonl').read_text().splitlines()]
    diagnostics=[row for row,keep in zip(d,m) if keep]
    primary=ph['servo_delay_ticks']==3 and ph['motor_tau']==.12
    movement=t['v']>.3
    z=dict(run=name,mode=mode,pose_delay_ms=ms,servo_delay_ms=ph['servo_delay_ticks']*20,
      motor_tau_ms=ph['motor_tau']*1000,compensation_ms=0 if r['compensation_s'] is None else r['compensation_s']*1000,
      primary=primary,requested_delay=ms in [0,50,100,150,180,200],reused=r['source']!='new_run',completed_laps=r['completed_laps'],
      stat_laps=len(lt),duration_s=float(a['t'][-1]+.02),lap_mean_s=avg(lt),lap_best_s=float(lt.min()) if len(lt) else None,
      lap_std_s=float(lt.std(ddof=1)) if len(lt)>1 else None,lateral_rmse_m=rmse(t['d']),
      lateral_rmse_moving_m=rmse(t['d'][movement]),lateral_max_m=float(np.max(abs(t['d']))) if len(t) else None,
      run_lateral_max_m=float(np.max(abs(a['d']))),stops_0p2_flying=stops(t,10) if len(t) else None,
      run_lateral_rmse_m=rmse(a['d']),run_lateral_rmse_moving_m=rmse(a['d'][a['v']>.3]),
      stops_0p5_full=stops(a,25),run_low_speed_pct=100*avg(a['v']<.1),
      mean_speed_mps=avg(t['v']),moving_speed_mean_mps=avg(t['v'][t['v']>.1]),max_speed_mps=float(t['v'].max()) if len(t) else None,
      solver_failure_pct=100*avg(t['solver_failure']),run_solver_failure_pct=100*avg(a['solver_failure']),
      combined_limit_over_pct=100*avg((t['a']/np.where(t['a']>=0,4.,6.))**2+(t['ay']/4.5)**2>1),
      motor_cap_pct=100*avg(t['cmd']>t['motor_cmd']+1e-6),
      lead_limited_pct=100*avg(np.array([x.get('lead_limited',False) for x in diagnostics])),
      solver_p99_ms=float(np.percentile([x['solver_ms'] for x in d],99)) if mode!='PP' else None)
    from collections import Counter
    counts=Counter(x['status'] for x in d)
    (OUT/f'{name}_status_counts.json').write_text(json.dumps(counts,indent=2))
    summary.append(z);sources.append(dict(run=name,source=r['source'],sha256_trace=hashlib.sha256((OUT/f'{name}_trace.csv').read_bytes()).hexdigest()))
    for i,tt in enumerate(r['lap_times']):laprows.append(dict(run=name,lap=i+1,time_s=tt,in_statistics=i>0))
    for ci,(b,e) in enumerate(zip(starts,ends),1):
        edges=np.linspace(s[b],s[e],4)
        for j,phase in enumerate(['entry','middle','exit']):
            sel=(t['s']>=edges[j])&(t['s']<edges[j+1])
            corners.append(dict(run=name,mode=mode,pose_delay_ms=ms,primary=primary,corner=ci,phase=phase,
             start_s=edges[j],end_s=edges[j+1],speed_mps=avg(t['v'][sel]),lateral_rmse_m=rmse(t['d'][sel]),samples=int(sel.sum())))
csvout('summary.csv',summary);csvout('corners.csv',corners);csvout('laps.csv',laprows);csvout('reuse_manifest.csv',sources)
styles={'PP':('#1f77b4','o'),'MPC ON':('#d95f02','s'),'MPC OFF':('#7544a2','^')}
fig,axs=plt.subplots(2,2,figsize=(13,8))
for mode,(color,marker) in styles.items():
    rr=sorted([r for r in summary if r['mode']==mode and r['primary'] and r['requested_delay']],key=lambda x:x['pose_delay_ms'])
    for ax,key in zip(axs.flat,['lap_mean_s','lateral_rmse_m','stops_0p5_full','stat_laps']):
        ax.plot([r['pose_delay_ms'] for r in rr],[r[key] if r[key] is not None else np.nan for r in rr],marker=marker,color=color,label=mode)
for ax,y in zip(axs.flat,['Mean flying lap (s)','Lateral RMSE (m)','Stops >=0.5 s, full run','Completed flying laps']):
    ax.set(xlabel='Injected pose delay (ms)',ylabel=y);ax.legend()
fig.suptitle('Pose-delay sweep | servo dead time 60 ms | motor tau 120 ms | MPC ON = fixed 180 ms prediction')
for r in summary:
    if r['primary'] and r['requested_delay'] and r['stat_laps']<10:
        axs[0,0].annotate(f"{r['mode']}: {r['stat_laps']}/10 laps",(r['pose_delay_ms'],.05),xycoords=('data','axes fraction'),rotation=90,fontsize=8)
save(fig,'01_pose_delay.png')
fig,axs=plt.subplots(2,3,figsize=(14,8))
for ci in [1,2]:
 for j,phase in enumerate(['entry','middle','exit']):
    ax=axs[ci-1,j]
    for mode,(color,marker) in styles.items():
        rr=sorted([r for r in corners if r['primary'] and r['pose_delay_ms'] in [0,50,100,150,180,200] and r['corner']==ci and r['phase']==phase and r['mode']==mode],key=lambda x:x['pose_delay_ms'])
        ax.plot([r['pose_delay_ms'] for r in rr],[r['speed_mps'] if r['speed_mps'] is not None else np.nan for r in rr],color=color,marker=marker,label=mode)
    ax.set(title=f'Corner {ci}: {phase}',xlabel='Pose delay (ms)',ylabel='Mean speed (m/s)');ax.legend()
fig.suptitle('Identical geometry-defined corner thirds for every controller')
save(fig,'02_corner_speeds.png')
fig,axs=plt.subplots(1,3,figsize=(14,4.5))
conditions=[(0,120,'Servo 0 / tau 120'),(60,120,'Servo 60 / tau 120'),(100,120,'Servo 100 / tau 120'),(60,450,'Servo 60 / tau 450')]
for mode,(color,marker) in styles.items():
    rr=[next((r for r in summary if r['pose_delay_ms']==100 and r['mode']==mode and r['servo_delay_ms']==se and r['motor_tau_ms']==ta),None) for se,ta,_ in conditions]
    for ax,key in zip(axs,['lap_mean_s','lateral_rmse_m','stops_0p5_full']):
        ax.plot(range(4),[r[key] if r and r[key] is not None else np.nan for r in rr],color=color,marker=marker,label=mode)
for ax,y in zip(axs,['Mean flying lap (s)','Lateral RMSE (m)','Stops >=0.5 s, full run']):
    ax.set_xticks(range(4),[label.replace(' / ','\n') for _,_,label in conditions],rotation=15);ax.set_ylabel(y);ax.legend()
fig.suptitle('Actuator sensitivity only | pose delay fixed at 100 ms | delays/tau in ms')
save(fig,'03_actuator_sensitivity.png')
realpath=ROOT/'docs/results/pp_vs_mpc_existing_20261126/runs.csv'
real=list(csv.DictReader(realpath.open()));csvout('real_existing_records.csv',real)
picked=[r for r in real if r['run'] in ['step45_mpc_20261126_182005','step45_mpc_20261126_184738']]
fig,axs=plt.subplots(1,3,figsize=(14,5))
simrow=next(r for r in summary if r['mode']=='MPC ON' and r['pose_delay_ms']==0 and r['primary'])
for ax,key,rkey,title in zip(axs,['lap_mean_s','lateral_rmse_moving_m','stops_0p5_full'],['lap_mean_s','lat_rmse_gbtrack_m','stops'],['Mean lap (s)','Moving lateral RMSE (m)','Stops >=0.5 s, full run']):
    vals=[simrow[key]]+[float(r[rkey]) for r in picked]
    ax.bar(range(3),vals,color=['#d95f02','#777777','#aaaaaa'])
    ax.set_xticks(range(3),['SIM final\n5p0, 0 ms','REAL Step51\n5p0, obstacles','REAL Step52\n7p0, obstacles']);ax.set_title(title)
    for i,v in enumerate(vals):ax.text(i,v,f'{v:.3f}',ha='center',va='bottom')
fig.suptitle('Context only: different code/conditions/duration; real RMSE uses estimated pose, not ground truth')
save(fig,'04_real_context_NOT_MATCHED.png')
fig,axs=plt.subplots(1,2,figsize=(12,5))
for mode,(color,marker) in styles.items():
    rr=sorted([r for r in summary if r['mode']==mode and r['primary'] and r['requested_delay']],key=lambda x:x['pose_delay_ms'])
    for ax,key in zip(axs,['combined_limit_over_pct','run_solver_failure_pct']):
        ax.plot([r['pose_delay_ms'] for r in rr],[r[key] for r in rr],color=color,marker=marker,label=mode)
for ax,y in zip(axs,['Ticks above assumed combined friction limit (%)','Solver failure ticks over full run (%)']):ax.set(xlabel='Pose delay (ms)',ylabel=y);ax.legend()
fig.suptitle('Model feasibility and solver audit: faster laps do not establish physically feasible performance')
save(fig,'05_model_audit.png')
print('Analyzed',len(summary),'runs')
