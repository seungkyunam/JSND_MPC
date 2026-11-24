"""Synthetic circle comparison; identical ideal plant, no hardware/ROS nodes."""
import json
from time import perf_counter
from collections import Counter
import numpy as np
from test_integrated_v2 import config, ROOT, IntegratedMPC
from controller.mpc.steering_mpc import SteeringMPC, SteeringConfig
from offline_support import load_controller, make_controller, cycle


def run(mode):
    c=config(); radius=3.; dt=c.dt
    theta=np.linspace(0,2*np.pi,600,endpoint=False)
    path=np.c_[radius*np.sin(theta),radius*(1-np.cos(theta)),np.ones(600),np.full(600,.5),
               radius*theta,np.full(600,1/radius),theta,np.zeros((600,2))]
    cls=load_controller(ROOT/'jsnd_int/racing_stack/controller/controller/combined/src/Controller.py')
    mpc=None
    if mode=='integrated': mpc=IntegratedMPC(c)
    if mode=='steering':
        mpc=SteeringMPC(SteeringConfig(wheelbase=c.wheelbase,max_steer=c.max_steer,
            max_steer_rate=c.max_steer_rate,max_speed=1.,horizon=15,max_iter=4000,
            solver_time_limit=.015,deadline=.02))
    ctrl=make_controller(cls=cls, wheelbase=c.wheelbase, lateral_controller='PP' if mode=='pp' else 'MPC',
                         lateral_mpc=mpc,mpc_speed_source='jsnd' if mode=='steering' else 'integrated')
    class CircleFrenet:
        def get_frenet(self,x,y):
            x,y=np.asarray(x),np.asarray(y)
            return radius*(np.arctan2(x,radius-y)%(2*np.pi)), radius-np.hypot(x,y-radius)
    ctrl.converter=CircleFrenet()
    state=np.zeros(4);previous=0.; times=[]; errors=[]; statuses=Counter();marks=[]; stops=0;was_stop=False
    for tick in range(6000):
        angle=np.arctan2(state[0],radius-state[1])%(2*np.pi)
        nearest=int(round(angle/(2*np.pi)*600))%600
        wp=path[(nearest+np.arange(-3,160))%600].copy()
        start=perf_counter()
        frenet=np.array([radius*angle,radius-np.hypot(state[0],state[1]-radius),state[3],0.])
        out=ctrl.main_loop('GB_TRACK',state[None,:3],wp,state[3],None,frenet,np.zeros(10),2*np.pi*radius)
        times.append((perf_counter()-start)*1000)
        speed,delta=out[0],out[3]
        if mode!='pp':
            statuses[ctrl.mpc_result.status]+=1
            stop=not ctrl.mpc_result.success
        else: stop=speed<=0
        stops+=int(stop and not was_stop);was_stop=stop
        # Identical ideal bicycle; speed servo has the existing lap_sim 0.12 s lag.
        v=state[3];state[:3]+=dt*v*np.array([np.cos(state[2]),np.sin(state[2]),np.tan(delta)/c.wheelbase])
        state[3]=max(0.,v+(speed-v)*dt/.12)
        errors.append(abs(np.hypot(state[0],state[1]-radius)-radius))
        if state[2]>2*np.pi*(len(marks)+1):
            marks.append((tick+1)*dt)
            if len(marks)==2: break
        if errors[-1]>1.:break
    return dict(mode=mode,scope='synthetic_R3_circle_1mps_ceiling_not_11_18_6',
        laps=len(marks),flying_lap_s=marks[1]-marks[0] if len(marks)>1 else None,
        max_error_m=max(errors),rms_error_m=float(np.sqrt(np.mean(np.square(errors)))),
        stops=stops,p99_ms=float(np.percentile(times,99)),max_ms=max(times),over20ms=sum(t>20 for t in times),statuses=dict(statuses))

if __name__=='__main__':
    print(json.dumps([run(m) for m in ['pp','steering','integrated']],indent=2))
