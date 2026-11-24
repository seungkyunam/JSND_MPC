"""No ROS/hardware. Exact-pose synthetic closed loop, real-car MPC config at 50Hz.
VESC time constant 0.12s is assumed; pose/servo delay and sensor noise are absent.
Run: PYTHONPATH=controller:controller/test:jsnd_int/tests_offline python3 <this file>
Output: JSONL, one row per scenario. This is not a real-car readiness certificate.
"""
from test_integrated_v2 import *
from controller.mpc.configuration import MPC_DEFAULTS,make_config
from collections import Counter
import yaml,json
failed=False
car=yaml.safe_load((ROOT/'jsnd_int/racing_stack/stack_master/config/mpc_car.yaml').read_text())['controller_manager']['ros__parameters']
for speed in [1.,2.]:
 for name in ['straight','s_curve','static_avoidance']:
  vals={k:car.get(k,v) for k,v in MPC_DEFAULTS.items()};vals.update(mpc_max_speed_mps=speed,mpc_deadline_s=.016,mpc_solver_time_limit_s=.008)
  c=make_config(vals,.324,True,50);m=IntegratedMPC(c)
  cls=load_controller(ROOT/'jsnd_int/racing_stack/controller/controller/combined/src/Controller.py')
  ctrl=make_controller(cls=cls,wheelbase=.324,lateral_controller='MPC',lateral_mpc=m)
  p=virtual_path(name)[::2];i=np.argmin(abs(p[:,0]));z=np.r_[p[i,:2],p[i,6],0.];statuses=Counter();errors=[];times=[]
  for tick in range(500):
   i=np.argmin(np.linalg.norm(p[:,:2]-z[:2],axis=1));wp=p[max(0,i-3):i+80]
   out=cycle(ctrl,wp,position=z[:3],speed=z[3]);r=ctrl.mpc_result;statuses[r.status]+=1;times.append(r.elapsed_s*1000)
   v=z[3];z[:3]+=.02*v*np.array([np.cos(z[2]),np.sin(z[2]),np.tan(out[3])/.324]);z[3]=max(0.,v+(out[0]-v)*.02/.12)
   if not r.success:ctrl.reset_mpc_speed_command()
   errors.append(np.min(np.linalg.norm(p[:,:2]-z[:2],axis=1)))
  failed |= any(k != 'solved' for k in statuses) or max(errors)>c.max_tracking_error
  print(json.dumps(dict(path=name,vmax=speed,status=dict(statuses),max_error=max(errors),p99_ms=float(np.percentile(times,99)),final_x=z[0])))

raise SystemExit(1 if failed else 0)
