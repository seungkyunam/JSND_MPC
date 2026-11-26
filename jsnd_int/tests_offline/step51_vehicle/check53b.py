import sys,json,rosbag2_py,importlib.util
from types import SimpleNamespace as NS
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def load(n,p):
    sp=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m
O=load('o',sys.argv[2]); N=load('n',sys.argv[3])
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
T={t.name:t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/state_machine/debug','/tracking/obstacles']))
obs=[];ev=[];prev=False;c={'tr':0,'old':0,'new':0,'newclear_overlap':0}
while r.has_next():
    n,d,t=r.read_next(); m=deserialize_message(d,get_message(T[n]))
    if n=='/tracking/obstacles': obs=list(m.obstacles); continue
    j=json.loads(m.data)
    t=t/1e9
    on=j['state']=='TRAILING' and bool((j.get('gb_free') or {}).get('is_free'))
    if on and not prev: ev.append([t,None,0])
    if on: ev[-1][2]+=1
    prev=on
    if not on: continue
    sm=NS(cur_d=j['cur_d'],cur_s=j['cur_s'],max_s=13.91,gb_ego_width_m=0.29,cur_obstacles_in_interest=obs)
    c['tr']+=1; c['old']+=O._gb_return_clear(sm); k=N._gb_return_clear(sm); c['new']+=k
    if k and ev[-1][1] is None: ev[-1][1]=t
print('TRAILING ticks with GB free:',c['tr'],'return-clear old',c['old'],'new',c['new'])
print([(e[2], None if e[1] is None else round(e[1]-e[0],2)) for e in ev])
