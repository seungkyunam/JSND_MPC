import json,hashlib,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];O=Path(__file__).parent/'results'
files=list(O.glob('*_run.json'));assert len(files)==31
cfg=None;reuse=0;fraction=0;done=0;timeouts=[];plantgroups={}
for p in files:
 r=json.loads(p.read_text());t=np.genfromtxt(O/(r['name']+'_trace.csv'),delimiter=',',names=True)
 assert all(np.isfinite(t[n]).all() for n in t.dtype.names)
 phy=r['physical']; key=(r['pose_delay_ms'],phy['servo_delay_ticks'],phy['motor_tau'])
 if key in plantgroups:assert phy==plantgroups[key]
 plantgroups[key]=phy
 assert t['v'][0]==0 and t['d'][0]<1e-6
 assert max(t['v'])<=phy['max_speed']+1e-8
 assert max(t['a'])<=4+1e-8 and min(t['a'])>=-6-1e-8
 assert max(abs(t['delta']))<=.49+1e-8 and max(abs(np.diff(t['delta'])/.02))<=6+1e-8
 assert np.allclose(np.diff(t['v']),t['a'][:-1]*.02,atol=1e-9)
 if r['controller']=='mpc':
  if cfg is None:cfg=r['config']
  assert r['config']==cfg
  assert r['compensation_s'] in [0,.18]
 if r['source']!='new_run':
  origin=Path(r['source'])
  for suffix in ['trace.csv','diagnostics.jsonl']:
   a=origin.with_name(origin.name.replace('run.json',suffix))
   b=O/(r['name']+'_'+suffix)
   assert hashlib.sha256(a.read_bytes()).digest()==hashlib.sha256(b.read_bytes()).digest()
  reuse+=1
 else:
  for a,b in [('pose_x','x'),('pose_y','y'),('pose_yaw','yaw')]:
   assert np.max(abs(t[a]-np.interp(t['t']-r['pose_delay_ms']/1000,t['t'],t[b])))<1e-9
  if r['pose_delay_ms'] in [50,150]:fraction+=1
 n=r['completed_laps'];done+=n
 assert len(r['lap_times'])==n
 if n<11:
  assert r['termination']=='300s_timeout' and abs(t['t'][-1]+.02-300)<1e-8
  timeouts.append(r['name'])
 m=(t['lap']>1)&(t['lap']<=n)
 assert abs(sum(r['lap_times'][1:])-.02*m.sum())<1e-8
assert reuse==6 and fraction==6
subprocess.run(['git','diff','--exit-code'],cwd=ROOT,check=True)
old=json.loads((ROOT/'benchmark_final/results/manifest.json').read_text())
for name,sha in old['files'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha,name
msg=f'''PASS: 31 cases; {reuse} reused traces/diagnostics byte-identical; {done} total completed laps.
PASS: 6 fractional-delay cases, plus all other new pose inputs match time interpolation to <1e-9.
PASS: same plant for paired conditions, unchanged optimizer config, OFF runtime prediction horizon only.
PASS: speed/acceleration/steering limits, state integration, finite traces, completed-lap accounting.
PASS: original benchmark manifest hashes and tracked production git diff unchanged.
TIMEOUTS (kept, not discarded): {timeouts}
NOT CERTIFIED: full delayed localization pipeline, tire friction/contact dynamics, real-car delay identification or matched real-vs-sim performance.
'''
(O/'validation.txt').write_text(msg);print(msg)
