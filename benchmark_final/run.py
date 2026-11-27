"""Read-only production controllers; reuse lap_sim track, builder and actuator model.
No ROS or hardware. Additional shared motor clamp; diagnostics never alter outputs.
"""
import os, sys, json, argparse, collections, dataclasses, hashlib
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'jsnd_int/racing_stack/controller'), str(ROOT/'controller/test'),
 str(ROOT/'race_utils/f110_utils/libs/frenet_conversion'), str(ROOT/'jsnd_int/tests_offline')]
import controller.mpc
controller.mpc.__path__.append(str(ROOT/'controller/controller/mpc'))
from offline_support import install_marker_shims
install_marker_shims()
os.environ['JSND_LAP_STACK_ROOT'] = str(ROOT/'jsnd_int/racing_stack/stack_master')
os.environ['JSND_LAP_MAP_ROOT'] = str(ROOT/'jsnd_int/racing_stack/stack_master/maps/11_18_6')
os.environ['JSND_LAP_BASE_CONFIG'] = str(ROOT/'stack_master/config')
import lap_sim as sim
import yaml
from PIL import Image

def run(ctrl, delay, profile, out):
    if (out/f'{ctrl}_{profile}_delay{delay}_run.json').exists(): return
    sim.REAL_DEADLINES = True
    c,p = sim.build(ctrl, profile, {'mpc_speed_source':'integrated',
        'mpc_history_prediction':True, 'mpc_heading_robust':False} if ctrl=='mpc' else {}, 50)
    # Old lap_sim.build predates Step 52: forward the exact production YAML value.
    if ctrl=='mpc':
        c.mpc_cmd_accel_lead_mps = p['mpc_cmd_accel_lead_mps']
    L=.324; dt=.02; vmax=23250/4614
    assert p.get('wheelbase',L)==L
    conv=c.converter
    x,y,yaw=map(float,sim.FULL[0,[0,1,6]])
    v=delta=t=total=0.; prev=float(conv.get_frenet([x],[y])[0][0])
    poses=collections.deque([(x,y,yaw)]*delay)
    servo=collections.deque([0.]*3) # existing Step51 60ms servo assumption, shared
    acc=np.zeros(10); records=[]; marks=[]; diagnostics=[]
    c.pose_prediction.clock=lambda:t
    mapdir=Path(os.environ['JSND_LAP_MAP_ROOT'])
    meta=yaml.safe_load((mapdir/'11_18_6.yaml').read_text())
    grid=np.array(Image.open(mapdir/meta['image']).convert('L'))
    def point_occupied(x,y):
        ix=int(np.floor((x-meta['origin'][0])/meta['resolution']))
        iy=grid.shape[0]-1-int(np.floor((y-meta['origin'][1])/meta['resolution']))
        return float(1-grid[iy,ix]/255)>meta['occupied_thresh'] if 0<=ix<grid.shape[1] and 0<=iy<grid.shape[0] else True
    for tick in range(15000): # 300s, no tuning/retry based on outcome
        s,d=map(float,conv.get_frenet([x],[y])[:,0])
        total+=(s-prev+sim.TRACK_LEN/2)%sim.TRACK_LEN-sim.TRACK_LEN/2; prev=s
        if total >= sim.TRACK_LEN*(len(marks)+1):
            marks.append(t)
            if len(marks)>=11: break
        wp=sim.local_wpnts(s)
        poses.append((x,y,yaw)); pose=np.array([poses.popleft()])
        c.yaw_rate=v*np.tan(delta)/L; c.pose_prediction.observe(t,c.yaw_rate)
        u=c.main_loop('GB_TRACK',pose,wp,v,None,np.array([s,d,v,0.]),acc,sim.TRACK_LEN)
        cmd,steer=float(u[0]),float(u[3]); motor=float(np.clip(cmd,0,vmax))
        diag={}; status='PP'; fail=False
        if ctrl=='mpc':
            r=c.mpc_result; status=r.status; fail=not r.success
            diag=dict(c.mpc_speed_diagnostics)
            diag['solver_ms']=r.elapsed_s*1000
            if c.lateral_mpc.plan is not None:
                diag['horizon_cap_min']=float(np.min(c.lateral_mpc.plan['caps']))
            if cmd<=0: c.reset_mpc_speed_command()
            c.record_applied_steering(steer)
        # Same discrete plant as lap_sim, adding physical motor speed saturation.
        servo.append(float(np.clip(steer,-sim.SERVO_LIMIT,sim.SERVO_LIMIT)))
        delta+=float(np.clip(servo.popleft()-delta,-sim.SERVO_RATE*dt,sim.SERVO_RATE*dt))
        a=float(np.clip((motor-v)/sim.VESC_TAU,-sim.VESC_DEC,sim.VESC_ACC))
        records.append(dict(t=t,lap=len(marks)+1,s=s,d=d,x=x,y=y,yaw=yaw,v=v,
            cmd=cmd,motor_cmd=motor,steer=steer,delta=delta,a=a,
            ay=v*v*abs(np.tan(delta))/L,kappa=float(np.interp(s,sim.FULL[:,4],sim.FULL[:,5])),
            plan_v=float(wp[0,2]),solver_failure=int(fail),point_occupied=int(point_occupied(x,y))))
        diagnostics.append({**diag,'t':t,'status':status})
        acc[1:]=acc[:-1]; acc[0]=a
        x+=dt*v*np.cos(yaw); y+=dt*v*np.sin(yaw); yaw+=dt*v*np.tan(delta)/L
        v=max(0.,v+a*dt); t=(tick+1)*dt
    name=f'{ctrl}_{profile}_delay{delay}'
    import csv
    with (out/f'{name}_trace.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=records[0]);w.writeheader();w.writerows(records)
    with (out/f'{name}_diagnostics.jsonl').open('w') as f:
        for row in diagnostics:f.write(json.dumps(row)+'\n')
    cfg=dataclasses.asdict(c.lateral_mpc.config) if ctrl=='mpc' else None
    summary=dict(name=name,controller=ctrl,profile=profile,pose_delay_ticks=delay,
       lap_marks=marks,lap_times=np.diff([0.]+marks).tolist(),completed_laps=len(marks),
       config=cfg,params=p,physical=dict(wheelbase=L,dt=dt,max_speed=vmax,
       accel=sim.VESC_ACC,decel=sim.VESC_DEC,steer=sim.SERVO_LIMIT,
       steer_rate=sim.SERVO_RATE,motor_tau=sim.VESC_TAU,servo_delay_ticks=3))
    (out/f'{name}_run.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:summary[k] for k in ['name','completed_laps','lap_times']}),flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--profile',default='5p0')
    ap.add_argument('--delays',nargs='+',type=int,default=[0,6,7]);a=ap.parse_args()
    out=Path(__file__).parent/'results'
    for delay in a.delays:
        for ctrl in ('pp','mpc'):run(ctrl,delay,a.profile,out)
