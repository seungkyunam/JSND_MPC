"""50Hz wall-clock synthetic controller test; no ROS and no workspace writes."""
import json
from collections import Counter
from pathlib import Path
from time import perf_counter, sleep
import numpy as np
from controller.mpc.bicycle import step
from controller.mpc.lateral_mpc import LateralMPC, MPCConfig
from offline_support import virtual_path, make_controller, cycle
from benchmark_mpc import stats

out=Path(__file__).resolve().parent
config=MPCConfig(wheelbase=.32,max_steer=.53,max_steer_rate=2.)
report={'period_ms':20.0,'scope':'900 real-time scheduled calls; synthetic rear-axle plant and mock Frenet; no ROS, no actuators; fixed measured forward speed on successful solves','paths':{}}
all_compute=[]
all_finish=[]
all_late=[]
for name,count,speed in [('straight',200,2.),('sharp_curve',100,1.5),('s_curve',250,2.),('static_avoidance',350,2.)]:
    path=virtual_path(name)
    start=int(np.argmin(abs(path[:,0]))) if name!='sharp_curve' else int(np.argmin(abs(path[:,4]-.6)))
    state=np.array([path[start,0],path[start,1]+.08,path[start,6]])
    ctrl=make_controller(lateral_controller='MPC',lateral_mpc=LateralMPC(config))
    compute=[]; lateness=[]; finish=[]; mpc=[]; status=Counter()
    origin=perf_counter()+.02
    for i in range(count):
        scheduled=origin+i*config.dt
        delay=scheduled-perf_counter()
        if delay>0:
            sleep(delay)
        started=perf_counter()
        output=cycle(ctrl,path,position=state,speed=speed)
        ended=perf_counter()
        compute.append(ended-started)
        lateness.append(max(0.,started-scheduled))
        finish.append(ended-scheduled)
        result=ctrl.mpc_result
        mpc.append(result.elapsed_s)
        status[result.status]+=1
        state=step(state,output[3],speed if result.success else 0.,config.dt,config.wheelbase)
    all_compute.extend(compute); all_finish.extend(finish); all_late.extend(lateness)
    report['paths'][name]={'controller_compute':stats(compute),'mpc_compute':stats(mpc),
                           'start_lateness':stats(lateness),'finish_from_scheduled':stats(finish),'statuses':dict(status)}
report['aggregate']={'controller_compute':stats(all_compute),'start_lateness':stats(all_late),'finish_from_scheduled':stats(all_finish)}
with (out/'periodic_50hz.json').open('x') as f:
    json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
