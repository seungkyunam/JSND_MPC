#!/usr/bin/env python3
"""Summarize recorded simulation speed phases; never connects to ROS."""
import argparse,csv,json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('result_dir',type=Path);p.add_argument('--map-dir',type=Path,required=True);a=p.parse_args()
    rows=list(csv.DictReader((a.result_dir/'trajectory.csv').open()))
    wp=json.loads((a.map_dir/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    path=np.array([[w['x_m'],w['y_m']] for w in wp]);curvature=np.abs([w['kappa_radpm'] for w in wp])
    t=np.array([float(r['stamp']) for r in rows]);t-=t[0]
    v=np.array([float(r['speed']) for r in rows]);err=np.array([float(r['error']) for r in rows])
    xy=np.array([[float(r['x']),float(r['y'])] for r in rows])
    nearest=np.argmin(np.sum((xy[:,None,:]-path)**2,axis=2),axis=1)
    # 0.4-second velocity differences suppress simulator/ROS sampling jitter.
    before=np.interp(np.maximum(0,t-.4),t,v);acc=(v-before)/.4
    k=curvature[nearest];ahead=curvature[(nearest+10)%len(path)]
    straight=k<.12;approach=(ahead>k+.15);exit_curve=(k>ahead+.15)
    report={'straight_acceleration_samples':int(np.sum(straight&(acc>.08))),
            'corner_approach_deceleration_samples':int(np.sum(approach&(acc<-.08))),
            'corner_exit_acceleration_samples':int(np.sum(exit_curve&(acc>.08))),
            'straight_mean_speed_mps':float(np.mean(v[straight])) if np.any(straight) else None,
            'corner_mean_speed_mps':float(np.mean(v[k>.4])) if np.any(k>.4) else None}
    report['all_three_speed_phases_observed']=all(report[x]>0 for x in ['straight_acceleration_samples','corner_approach_deceleration_samples','corner_exit_acceleration_samples'])
    (a.result_dir/'speed_phases.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(11,8),constrained_layout=True)
    ax[0].plot(path[:,0],path[:,1],'k--',label='Global path');ax[0].plot(xy[:,0],xy[:,1],label='ROS simulator');ax[0].axis('equal');ax[0].legend()
    ax[1].plot(t,v,label='Measured speed');ax[1].plot(t,[float(r['command_speed']) for r in rows],alpha=.6,label='MPC speed');ax[1].set_ylabel('m/s');ax[1].legend()
    ax[2].plot(t,err);ax[2].set_ylabel('Path error (m)');ax[2].set_xlabel('Time (s)')
    fig.savefig(a.result_dir/'trajectory.png',dpi=160)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
