import sys,json, rosbag2_py, numpy as np
from types import SimpleNamespace as NS
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from controller.combined.src.Controller import Controller as C
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/controller/mpc_diagnostics']))
c=NS(mpc_cmd_max_lead_mps=0.5,mpc_cmd_accel_lead_mps=1.2,mpc_cmd_lead=0.5,ACCEL_LEAD_RAMP_PER_TICK_MPS=C.ACCEL_LEAD_RAMP_PER_TICK_MPS,_plan_never_brakes=C._plan_never_brakes)
def newlead(st,acc,p):
    lead=c.mpc_cmd_max_lead_mps
    if st in('solved','solved_late') and acc>0 and c._plan_never_brakes(p): lead=1.2
    elif st in('solved','solved_late') and acc>0 and c.mpc_cmd_lead>lead: lead=max(lead,c.mpc_cmd_lead-c.ACCEL_LEAD_RAMP_PER_TICK_MPS)
    c.mpc_cmd_lead=lead; return lead
po=pn=None; bo=bn=0; mo=mn=0
while r.has_next():
    _,d,t=r.read_next(); j=json.loads(deserialize_message(d,String).data); p=j.get('predicted_speed_mps')
    if not p: c.mpc_cmd_lead=0.5; po=pn=None; continue
    lo=j['lead_mps']; ln=newlead(j['status'],j['applied_accel_mps2'],p)
    acc=j['status'] in('solved','solved_late') and j['applied_accel_mps2']>0
    if acc and po is not None:
        mo=max(mo,po-lo); mn=max(mn,pn-ln); bo+=po-lo>0.3; bn+=pn-ln>0.3
    po,pn=lo,ln
print(f'accelerating ticks: lead step >0.3 old {bo} new {bn}; max step old {mo:.2f} new {mn:.2f}')
