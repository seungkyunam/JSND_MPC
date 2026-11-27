#!/usr/bin/env python3
"""Create compact tables and comparison plots from run_trial.py artifacts."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT/'results'


def load(path):
    return json.loads((path/'summary.json').read_text())


def write_csv(path, rows):
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)


horizons = [load(RESULTS/f'horizon_n{n}') for n in (10, 15, 20, 25)]
horizon_rows = []
for row in horizons:
    horizon_rows.append(dict(N=row['config']['horizon'], preview_dt_s=row['config']['preview_dt'],
        horizon_s=.02+(row['config']['horizon']-1)*row['config']['preview_dt'],
        best_lap_s=row['best_lap_s'], mean_lap_s=row['mean_lap_s'], lap_std_s=row['lap_std_s'],
        tracking_rmse_m=row['tracking_rmse_m'], solver_p50_ms=row['solver_ms']['p50'],
        solver_p95_ms=row['solver_ms']['p95'], solver_p99_ms=row['solver_ms']['p99'],
        solver_max_ms=row['solver_ms']['max'], cycle_overrun_pct=row['cycle_overrun_pct'],
        failure_pct=row['solver_failure_pct'], stop_events=row['stop_events']))
write_csv(RESULTS/'horizon_summary.csv', horizon_rows)

final = []
for scenario in ('ideal', 'realistic'):
    for profile in ('pp', 'legacy057', 'final'):
        final.append(load(RESULTS/f'final_{profile}_{scenario}'))
final_rows = []
for row in final:
    final_rows.append(dict(scenario=row['scenario'], controller=row['profile'],
        completed_laps=row['completed_laps'], statistic_laps=len(row['lap_times_s']),
        best_lap_s=row['best_lap_s'], mean_lap_s=row['mean_lap_s'], lap_std_s=row['lap_std_s'],
        max_speed_mps=row['max_speed_mps'], mean_speed_mps=row['mean_speed_mps'],
        tracking_rmse_m=row['tracking_rmse_m'], solver_p50_ms=row['solver_ms']['p50'],
        solver_p95_ms=row['solver_ms']['p95'], solver_p99_ms=row['solver_ms']['p99'],
        solver_max_ms=row['solver_ms']['max'], cycle_overrun_pct=row['cycle_overrun_pct'],
        failure_pct=row['solver_failure_pct'], stop_events=row['stop_events'],
        occupied_samples=row['occupied_samples'], lateral_limit_violations=row['lateral_limit_violations'],
        combined_limit_violation_pct=row['combined_limit_violation_pct']))
write_csv(RESULTS/'final_summary.csv', final_rows)

plt.style.use('ggplot')
fig, axes = plt.subplots(2, 2, figsize=(11, 7))
n = [r['N'] for r in horizon_rows]
for ax, key, ylabel in zip(axes.flat,
        ('mean_lap_s', 'tracking_rmse_m', 'solver_p99_ms', 'solver_max_ms'),
        ('Mean lap [s]', 'Tracking RMSE [m]', 'OSQP P99 [ms]', 'OSQP max [ms]')):
    ax.plot(n, [r[key] for r in horizon_rows], marker='o', linewidth=2)
    ax.set(xlabel='Prediction stages N', ylabel=ylabel, xticks=n)
fig.suptitle('One-second horizon sweep (11 laps, first excluded)')
fig.tight_layout(); fig.savefig(RESULTS/'horizon_comparison.png', dpi=160); plt.close(fig)

labels = ['Tuned PP', 'MPC 0.57 s', 'MPC 1.00 s']
profiles = ('pp', 'legacy057', 'final')
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
x = np.arange(3); width = .36
for offset, scenario in ((-.18, 'ideal'), (.18, 'realistic')):
    rows = [next(r for r in final_rows if r['scenario'] == scenario and r['controller'] == p) for p in profiles]
    axes[0, 0].bar(x+offset, [r['mean_lap_s'] for r in rows], width, label=scenario)
    axes[0, 1].bar(x+offset, [r['tracking_rmse_m'] for r in rows], width)
    axes[1, 0].bar(x+offset, [r['mean_speed_mps'] for r in rows], width)
    axes[1, 1].bar(x+offset, [r['combined_limit_violation_pct'] for r in rows], width)
for ax, ylabel in zip(axes.flat, ('Mean lap [s]', 'Tracking RMSE [m]',
                                  'Mean speed [m/s]', 'Combined-limit violation [%]')):
    ax.set_xticks(x, labels, rotation=10); ax.set_ylabel(ylabel)
axes[0, 0].legend()
fig.suptitle('Final 11-lap comparison on 11_18_6')
fig.tight_layout(); fig.savefig(RESULTS/'final_comparison.png', dpi=160); plt.close(fig)

fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
for ax, scenario in zip(axes, ('ideal', 'realistic')):
    for profile, label in zip(profiles, labels):
        data = np.genfromtxt(RESULTS/f'final_{profile}_{scenario}'/'trace.csv', delimiter=',', names=True,
                             dtype=None, encoding=None)
        mask = data['lap'] == 11
        order = np.argsort(data['s'][mask])
        ax.plot(data['s'][mask][order], data['speed'][mask][order], label=label, alpha=.85)
    ax.set(title=scenario, xlabel='Track s [m]', ylabel='Speed [m/s]')
axes[1].legend(); fig.tight_layout(); fig.savefig(RESULTS/'speed_profiles.png', dpi=160); plt.close(fig)

print(RESULTS/'horizon_summary.csv')
print(RESULTS/'final_summary.csv')
