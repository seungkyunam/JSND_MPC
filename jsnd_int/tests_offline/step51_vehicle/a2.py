from common import *
C=collections
diag=D['/controller/mpc_diagnostics']; sm=D['/state_machine']; es=D['/simple_mux/estop_state']; cmd=D['/vesc/ackermann_cmd']
core=np.array(D['/vesc/sensors/core'])
for lap in range(1,15):
    a,b=B[lap-1],B[lap]
    st=C.Counter(x[1]['status'].split(':')[0] if not x[1]['status'].startswith(('STOP','osqp','Value','nonlin','env','trail','dead')) else x[1]['status'] for x in diag if a<=x[0]<b)
    stops={k:v for k,v in st.items() if not k.startswith(('solved','tracking','braking_recovery','plan_cont'))}
    sms=C.Counter(x[1] for x in sm if a<=x[0]<b)
    ess=C.Counter(x[1].split(':')[0]+':'+x[1].split(':')[1][:20] for x in es if a<=x[0]<b)
    zc=sum(1 for x in cmd if a<=x[0]<b and x[1]==0.0); nc=sum(1 for x in cmd if a<=x[0]<b)
    m=(core[:,0]>=a)&(core[:,0]<b)
    nt=sum(1 for t in ttc if a<=t<b)
    print(f"L{lap:2d} {b-a:6.2f}s TTC={nt} cmd0={zc/nc*100:4.1f}% vmaxERPM={core[m,1].max():.0f}\n   SM={dict(sms)}\n   ES={dict(ess)}\n   MPCbad={stops}")
