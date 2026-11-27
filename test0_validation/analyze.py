#!/usr/bin/env python3
"""Aggregate the saved test0 trials and create tables, figures and diagnostics."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT/'stack_master/maps/test0'
LABELS = {'pp_pp':'Tuned PP', 'mpc_legacy057':'Legacy MPC 0.57 s', 'mpc_final':'Improved MPC 1.0 s (N=10)'}
SCENARIOS = {'clear':'No obstacle', 'obstacles':'2 static obstacles',
             'obstacles_realistic':'2 obstacles + delay/noise'}
COLORS = {'pp_pp':'#2ca02c', 'mpc_legacy057':'#1f77b4', 'mpc_final':'#d62728'}


def load(results):
    records = []
    for directory in sorted(p for p in results.iterdir() if p.is_dir() and (p/'summary.json').exists()):
        d = json.loads((directory/'summary.json').read_text())
        key, scenario = directory.name.split('__', 1)
        passages = d.get('passages') or []
        clearance = [p['min_clearance_m'] for p in passages if p.get('min_clearance_m') is not None]
        before = [p['speed_before_mps'] for p in passages if p.get('speed_before_mps') is not None]
        after = [p['speed_after_mps'] for p in passages if p.get('speed_after_mps') is not None]
        records.append(dict(case=directory.name, controller=key, controller_label=LABELS[key],
            scenario=scenario, scenario_label=SCENARIOS[scenario], completed_laps=d['completed_laps'],
            terminal_stop=d.get('terminal_stop', False), best_lap_s=d.get('best_lap_s'),
            mean_lap_s=d.get('mean_lap_s'), worst_lap_s=max(d.get('lap_times_s') or [np.nan]),
            lap_std_s=d.get('lap_std_s'), mean_speed_mps=d.get('mean_speed_mps'), max_speed_mps=d.get('max_speed_mps'),
            tracking_rmse_m=d.get('tracking_rmse_m'), avoidance_success_pct=d.get('avoidance_success_pct'),
            min_body_clearance_m=min(clearance) if clearance else np.nan,
            mean_speed_before_mps=np.mean(before) if before else np.nan,
            mean_speed_after_mps=np.mean(after) if after else np.nan,
            collision_events=d.get('collision_events'), stop_events=d.get('stop_events'),
            physical_violation_pct=d.get('physical_violation_pct'), solver_failure_pct=d.get('solver_failure_pct'),
            cycle_overrun_pct=d.get('cycle_overrun_pct'), osqp_p50_ms=d['solver_ms']['p50'],
            osqp_p95_ms=d['solver_ms']['p95'], osqp_p99_ms=d['solver_ms']['p99'], osqp_max_ms=d['solver_ms']['max']))
    return pd.DataFrame(records)


def failure_diagnostics(results):
    out = {}
    for directory in sorted(results.glob('mpc_*')):
        if not directory.is_dir(): continue
        trace = pd.read_csv(directory/'trace.csv')
        active = trace[(trace.lap > 1) & (trace.avoidance_active == 1)]
        inactive = trace[(trace.lap > 1) & (trace.avoidance_active == 0)]
        failures = trace[(trace.lap > 1) & (trace.solver_failure == 1)]
        statuses = Counter(failures.status)
        out[directory.name] = dict(
            total_failure_count=int(len(failures)),
            failure_status_counts=dict(statuses),
            failure_in_avoidance_pct=float(100*failures.avoidance_active.mean()) if len(failures) else 0.0,
            avoidance_failure_pct=float(100*active.solver_failure.mean()) if len(active) else None,
            global_path_failure_pct=float(100*inactive.solver_failure.mean()) if len(inactive) else None,
            avoidance_tracking_rmse_m=float(np.sqrt(np.mean(active.d**2))) if len(active) else None,
            avoidance_abs_d_max_m=float(active.d.abs().max()) if len(active) else None,
            command_steer_abs_p99_rad=float(active.command_steer.abs().quantile(.99)) if len(active) else None,
        )
    return out


def comparison_figure(df, out):
    metrics = [('mean_lap_s','Mean lap [s]'), ('mean_speed_mps','Mean speed [m/s]'),
               ('tracking_rmse_m','Tracking RMSE [m]'), ('avoidance_success_pct','Avoidance success [%]'),
               ('min_body_clearance_m','Min body clearance [m]'), ('solver_failure_pct','Controller failure [%]')]
    fig, axes = plt.subplots(2,3,figsize=(15,8))
    x=np.arange(3); width=.25
    for ax,(column,title) in zip(axes.flat,metrics):
        for j,key in enumerate(LABELS):
            values=[]
            for scenario in SCENARIOS:
                row=df[(df.controller==key)&(df.scenario==scenario)]
                values.append(float(row.iloc[0][column]) if len(row) and pd.notna(row.iloc[0][column]) else np.nan)
            ax.bar(x+(j-1)*width,values,width,label=LABELS[key],color=COLORS[key])
        ax.set_xticks(x, ['Clear','Obstacles','Delay/noise'],rotation=10); ax.set_title(title); ax.grid(axis='y',alpha=.25)
    axes[0,0].legend(fontsize=8); fig.tight_layout(); fig.savefig(out,dpi=180); plt.close(fig)


def map_figure(results, out):
    meta=yaml.safe_load((MAP/'test0.yaml').read_text()); image=np.asarray(Image.open(MAP/meta['image']).convert('L'))
    origin=meta['origin']; extent=[origin[0],origin[0]+image.shape[1]*meta['resolution'],origin[1],origin[1]+image.shape[0]*meta['resolution']]
    obstacles=[(15.18,'corner'),(28.97,'straight')]
    raw=json.loads((MAP/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    obsxy=[]
    for s,label in obstacles:
        p=min(raw,key=lambda q:abs(q['s_m']-s)); obsxy.append((p['x_m'],p['y_m'],label))
    fig,axes=plt.subplots(3,3,figsize=(15,12),sharex=True,sharey=True)
    for row,scenario in enumerate(SCENARIOS):
        for col,key in enumerate(LABELS):
            ax=axes[row,col]; trace=pd.read_csv(results/f'{key}__{scenario}'/'trace.csv'); trace=trace[trace.lap>1].iloc[::3]
            ax.imshow(image,cmap='gray',origin='upper',extent=extent,vmin=0,vmax=255)
            sc=ax.scatter(trace.x,trace.y,c=trace.speed,s=2,cmap='turbo',vmin=0,vmax=5.1)
            if scenario!='clear':
                for x,y,label in obsxy:
                    ax.add_patch(plt.Circle((x,y),.15,color='magenta',ec='black',lw=.6)); ax.text(x,y+.28,label,fontsize=7,ha='center')
            ax.set_title(f'{LABELS[key]}\n{SCENARIOS[scenario]}'); ax.set_aspect('equal'); ax.grid(alpha=.1)
    fig.colorbar(sc,ax=axes.ravel().tolist(),label='Speed [m/s]',shrink=.75); fig.savefig(out,dpi=180,bbox_inches='tight'); plt.close(fig)


def speed_figure(results, out):
    fig,axes=plt.subplots(1,2,figsize=(14,5),sharey=True)
    for ax,scenario in zip(axes,['obstacles','obstacles_realistic']):
        for key in LABELS:
            trace=pd.read_csv(results/f'{key}__{scenario}'/'trace.csv'); trace=trace[trace.lap>1]
            bins=np.linspace(0,41.652,209); idx=np.digitize(trace.s,bins)
            centers=(bins[:-1]+bins[1:])/2
            med=np.asarray([trace.speed[idx==i].median() for i in range(1,len(bins))])
            ax.plot(centers,med,label=LABELS[key],color=COLORS[key],lw=1.8)
        for s in (15.18,28.97): ax.axvline(s,color='magenta',ls='--',lw=1)
        ax.set_title(SCENARIOS[scenario]); ax.set_xlabel('Track s [m]'); ax.grid(alpha=.25)
    axes[0].set_ylabel('Median speed [m/s]'); axes[0].legend(fontsize=8); fig.tight_layout(); fig.savefig(out,dpi=180); plt.close(fig)


def failure_figure(results, out):
    fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True)
    for key in ('mpc_legacy057','mpc_final'):
        trace=pd.read_csv(results/f'{key}__obstacles'/'trace.csv'); trace=trace[trace.lap>1]
        axes[0].scatter(trace.s,trace.d,c=np.where(trace.solver_failure,1,0),s=4,cmap='coolwarm',vmin=0,vmax=1,alpha=.55,label=LABELS[key])
        bins=np.linspace(0,41.652,84); idx=np.digitize(trace.s,bins)
        rate=[100*trace.solver_failure[idx==i].mean() if np.any(idx==i) else np.nan for i in range(1,len(bins))]
        axes[1].plot((bins[:-1]+bins[1:])/2,rate,label=LABELS[key],color=COLORS[key])
    for ax in axes:
        for s in (15.18,28.97): ax.axvline(s,color='magenta',ls='--',lw=1)
        ax.grid(alpha=.25); ax.legend(fontsize=8)
    axes[0].set_ylabel('Frenet d [m]\n(red = failed sample)'); axes[1].set_ylabel('Failure rate [%]'); axes[1].set_xlabel('Track s [m]')
    fig.tight_layout(); fig.savefig(out,dpi=180); plt.close(fig)


def manifest():
    files=['test0.png','test0.yaml','global_waypoints.json','speed_scaling.yaml','ot_sectors.yaml']
    return {name:dict(bytes=(MAP/name).stat().st_size,sha256=hashlib.sha256((MAP/name).read_bytes()).hexdigest()) for name in files}


def main():
    p=argparse.ArgumentParser(); p.add_argument('--results',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    df=load(a.results); df.to_csv(a.output/'comparison.csv',index=False,float_format='%.9g')
    (a.output/'comparison.json').write_text(json.dumps(df.replace({np.nan:None}).to_dict('records'),indent=2)+'\n')
    diag=failure_diagnostics(a.results); (a.output/'failure_diagnostics.json').write_text(json.dumps(diag,indent=2)+'\n')
    (a.output/'map_asset_manifest.json').write_text(json.dumps(manifest(),indent=2)+'\n')
    comparison_figure(df,a.output/'comparison_metrics.png'); map_figure(a.results,a.output/'map_trajectories.png')
    speed_figure(a.results,a.output/'speed_profiles.png'); failure_figure(a.results,a.output/'mpc_failure_locations.png')
    print(df.to_string(index=False))
if __name__=='__main__': main()
