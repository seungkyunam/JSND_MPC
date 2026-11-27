#!/usr/bin/env python3
"""Build the final comparison table and figures from saved runs only."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'test0_mpc_improvement/final_validation'
OUT = ROOT / 'test0_mpc_improvement/analysis'
OUT.mkdir(exist_ok=True)

RUNS = {
    'PP N/A': RESULTS/'test0_baseline_ideal_pp_clearance035',
    'MPC N10 nonuniform': RESULTS/'test0_baseline_ideal_improved1s_qpos20',
    'MPC N15 nonuniform': RESULTS/'test0_baseline_ideal_n15u_qpos20',
    'MPC N10 alternate': RESULTS/'test0_alternate_ideal',
}

rows = []
for name, directory in RUNS.items():
    s = json.loads((directory/'summary.json').read_text())
    clearance = min((p['min_clearance_m'] for p in s.get('passages', [])
                     if p['min_clearance_m'] is not None), default=None)
    rows.append(dict(name=name, completed_laps=s['completed_laps'], best_lap_s=s['best_lap_s'],
        mean_lap_s=s['mean_lap_s'], lap_std_s=s['lap_std_s'], mean_speed_mps=s['mean_speed_mps'],
        max_speed_mps=s['max_speed_mps'], tracking_rmse_m=s['tracking_rmse_m'],
        avoidance_success_pct=s['avoidance_success_pct'], min_body_clearance_m=clearance,
        collision_events=s['collision_events'], stop_events=s['stop_events'],
        terminal_stop=s['terminal_stop'], physical_violation_pct=s['physical_violation_pct'],
        failure_pct=s['solver_failure_pct'], cycle_overrun_pct=s['cycle_overrun_pct'],
        solver_p50_ms=s['solver_ms']['p50'], solver_p95_ms=s['solver_ms']['p95'],
        solver_p99_ms=s['solver_ms']['p99'], solver_max_ms=s['solver_ms']['max']))

with (OUT/'final_comparison.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
(OUT/'final_comparison.json').write_text(json.dumps(rows, indent=2)+'\n')

fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
labels = [r['name'].replace(' nonuniform', '\nnonuniform') for r in rows[:3]]
axes[0].bar(labels, [r['mean_lap_s'] or 0 for r in rows[:3]], color=['#777','#2878b5','#d9534f'])
axes[0].set_ylabel('Mean lap time [s]'); axes[0].set_title('11-lap validation (lap 1 excluded)')
axes[1].bar(labels, [r['failure_pct'] or 0 for r in rows[:3]], color=['#777','#2878b5','#d9534f'])
axes[1].set_ylabel('Control failure [%]'); axes[1].set_title('N15 terminally stopped at lap 9')
axes[2].bar(labels, [r['min_body_clearance_m'] or 0 for r in rows[:3]], color=['#777','#2878b5','#d9534f'])
axes[2].set_ylabel('Minimum body clearance [m]'); axes[2].set_title('0.35 m planner offset')
for ax in axes: ax.tick_params(axis='x', labelrotation=15); ax.grid(axis='y', alpha=.25)
fig.tight_layout(); fig.savefig(OUT/'candidate_comparison.png', dpi=180); plt.close(fig)

map_dir = ROOT/'stack_master/maps/test0'
meta = yaml.safe_load((map_dir/'test0.yaml').read_text())
image = np.asarray(Image.open(map_dir/meta['image']).convert('L'))
extent = [meta['origin'][0], meta['origin'][0]+image.shape[1]*meta['resolution'],
          meta['origin'][1], meta['origin'][1]+image.shape[0]*meta['resolution']]
fig, axes = plt.subplots(1, 2, figsize=(13, 6))
for ax, (name, directory) in zip(axes, list(RUNS.items())[:2]):
    d = pd.read_csv(directory/'trace.csv'); d = d[d.lap > 1]
    ax.imshow(image, cmap='gray', origin='upper', extent=extent)
    sc=ax.scatter(d.x, d.y, c=d.speed, s=2, cmap='turbo', vmin=0, vmax=5)
    summary=json.loads((directory/'summary.json').read_text())
    for obs in summary['assumptions']['obstacle_placements']:
        wp=json.loads((map_dir/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
        i=int(np.argmin([abs(p['s_m']-obs['s']) for p in wp])); p=wp[i]
        ax.add_patch(plt.Circle((p['x_m'],p['y_m']),obs['size']/2,color='red',alpha=.75))
    ax.set_title(name); ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
fig.colorbar(sc, ax=axes, label='speed [m/s]', shrink=.75)
fig.savefig(OUT/'pp_vs_mpc_trajectories.png', dpi=180, bbox_inches='tight'); plt.close(fig)

# Aggregate the three required realistic seeds.
seed_rows=[]
for p in sorted(RESULTS.glob('test0_baseline_realistic_seed_*/summary.json')):
    s=json.loads(p.read_text()); seed_rows.append({
        'seed':s['assumptions']['seed'],'mean_lap_s':s['mean_lap_s'],
        'avoidance_success_pct':s['avoidance_success_pct'],'collision_events':s['collision_events'],
        'stop_events':s['stop_events'],'physical_violation_pct':s['physical_violation_pct'],
        'failure_pct':s['solver_failure_pct'],'cycle_overrun_pct':s['cycle_overrun_pct']})
with (OUT/'realistic_seeds.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=seed_rows[0]);w.writeheader();w.writerows(seed_rows)
