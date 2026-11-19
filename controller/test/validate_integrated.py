"""Reproducible kinematic closed-loop test (NOT ROS or physical validation)."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from controller.mpc.integrated_mpc import IntegratedConfig, IntegratedMPC


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    wp = json.loads((args.map_dir/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    path = np.array([[w['x_m'], w['y_m'], w['vx_mps']] for w in wp])
    if np.linalg.norm(path[0,:2]-path[-1,:2]) < 1e-6: path=path[:-1]
    # Match the isolated test0 simulation budget; this is still an ideal
    # kinematic supplement, not a ROS or physical-vehicle claim.
    c = IntegratedConfig(wheelbase=.3302, max_steer=.4189, max_steer_rate=3.2,
                         horizon=10, max_speed=2.0, solver_time_limit=.015,
                         deadline=.02, max_iter=1000)
    mpc=IntegratedMPC(c)
    state=np.array([*path[0,:2], np.arctan2(*(path[1,:2]-path[0,:2])[::-1]), 0.])
    previous=0.; progress=0.; last=0; laps=[]; last_lap=0.; rows=[]; failures={}
    for tick in range(15000):
        nearest=int(np.argmin(np.linalg.norm(path[:,:2]-state[:2],axis=1)))
        advance=(nearest-last+len(path)//2)%len(path)-len(path)//2
        progress+=advance; last=nearest
        if progress >= (len(laps)+1)*len(path):
            laps.append(tick*c.dt-last_lap);last_lap=tick*c.dt
            if len(laps)==3: break
        indices=(nearest+np.arange(-3, min(len(path)-3,200)))%len(path)
        result=mpc.solve(state,path[indices],previous)
        if not result.success:
            failures[result.status]=failures.get(result.status,0)+1
        speed=result.speed;steer=result.steering
        error=float(np.min(np.linalg.norm(path[:,:2]-state[:2],axis=1)))
        rows.append((tick*c.dt,*state,speed,steer,result.acceleration,error,result.elapsed_s,result.status))
        # Ideal velocity tracking and nonlinear rear-axle bicycle plant.
        v=.5*(state[3]+speed)
        state[:3]+=c.dt*np.array([v*np.cos(state[2]),v*np.sin(state[2]),v*np.tan(steer)/c.wheelbase])
        state[3]=speed;previous=steer
        if error > 1.: break
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'kinematic.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['t','x','y','yaw','v','command_v','steer','acceleration','error','solve_s','status']);writer.writerows(rows)
    data=np.array([r[:10] for r in rows],float)
    report=dict(kind='ideal_kinematic_not_ROS',map=str(args.map_dir),laps_s=laps,
                continuous_success=len(laps)==3 and not failures,
                failures=failures,rmse_m=float(np.sqrt(np.mean(data[:,8]**2))),
                max_error_m=float(np.max(data[:,8])),max_speed_mps=float(np.max(data[:,4])),
                solve_p99_ms=float(np.percentile(data[:,9],99)*1000))
    (args.output/'kinematic.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if len(laps)==3 and not failures else 1

if __name__=='__main__': raise SystemExit(main())
