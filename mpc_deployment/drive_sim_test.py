#!/usr/bin/env python3
"""Isolated simulation ONLY: exercise normal joystick/mux authority and record laps."""
import argparse, csv, json, math, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import sim_probe

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('build'); parser.add_argument('--map',default='test0')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=180)
    args=parser.parse_args(); env=sim_probe.launch_environment(os.environ)
    import rclpy
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String, Bool, Float32
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    sim_probe.rclpy=rclpy;sim_probe.GetParameters=GetParameters
    root=Path(__file__).resolve().parents[1]
    wp=json.loads((root/'stack_master/maps'/args.map/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    path=np.array([[p['x_m'],p['y_m']] for p in wp]);path=path[:-1] if np.linalg.norm(path[0]-path[-1])<1e-6 else path
    seg=np.roll(path,-1,axis=0)-path; lengths=np.linalg.norm(seg,axis=1);arc=np.r_[0,np.cumsum(lengths)]
    args.output.mkdir(parents=True,exist_ok=False)
    process=None;node=None;rows=[];laps=[];failures={};result={'map':args.map,'success':False}
    latest={'odom':None,'status':'waiting','collision':False,'collision_count':0,'active':False,'command':(0.,0.,0.)}
    solver_ms=[]; control_stamps=[]
    log=(args.output/'launch.log').open('w')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def joy(auto):
        msg=Joy();msg.header.stamp=node.get_clock().now().to_msg();msg.axes=[0.]*8;msg.buttons=[0]*12;msg.buttons[5 if auto else 4]=1;pub.publish(msg)
    try:
        process=subprocess.Popen(['bash',str(root/'run_mpc.sh'),args.build,args.map,'mpc'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        start=time.monotonic()
        while 'Simulation only. ROS domain checks' not in (args.output/'launch.log').read_text():
            if process.poll() is not None or time.monotonic()-start>30:raise RuntimeError('launch preflight failed')
            time.sleep(.2)
        rclpy.init();node=rclpy.create_node('integrated_sim_trial')
        node.create_subscription(Odometry,'/car_state/odom',lambda m:latest.update(odom=m),qos_profile_sensor_data)
        def status(m):
            latest['status']=m.data
            if latest['active'] and m.data!='solved':failures[m.data]=failures.get(m.data,0)+1
        node.create_subscription(String,'/controller/mpc_status',status,10)
        node.create_subscription(Float32,'/controller/mpc_elapsed_ms',lambda m: solver_ms.append(float(m.data)),10)
        def collision(m):latest.update(collision=latest['collision'] or m.data,collision_count=latest['collision_count']+1)
        node.create_subscription(Bool,'/sim_test/wall_collision',collision,10)
        node.create_subscription(AckermannDriveStamped,'/vesc/high_level/ackermann_cmd',lambda m:latest.update(command=(m.drive.speed,m.drive.steering_angle,m.drive.acceleration)),10)
        pub=node.create_publisher(Joy,'/joy',10)
        reset=node.create_publisher(PoseWithCovarianceStamped,'/sim/initialpose',10)
        spin(30)
        if sim_probe.parameters(node,'controller_manager',['sim','lateral_controller']) != {'sim':True,'lateral_controller':'MPC'}:raise RuntimeError('not MPC sim')
        if {e.node_name for e in node.get_publishers_info_by_topic('/car_state/odom')}!={'bridge'}:raise RuntimeError('unexpected odom owner')
        if {e.node_name for e in node.get_subscriptions_info_by_topic('/vesc/ackermann_cmd')}!={'bridge'}:raise RuntimeError('non-simulator command subscriber')
        joy(False);spin(.5)
        start_index=0
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.pose.position.x=float(path[start_index,0]);msg.pose.pose.position.y=float(path[start_index,1])
        yaw=math.atan2(seg[start_index,1],seg[start_index,0]);msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        reset.publish(msg);spin(2)
        if latest['odom'] is None:raise RuntimeError('missing odometry')
        xy=latest['odom'].pose.pose.position
        if math.hypot(xy.x-path[0,0],xy.y-path[0,1])>.1:raise RuntimeError('reset failed')
        latest['collision']=False
        latest['active']=True
        progress=0.;previous=None;last_stamp=None;lap_start=None;start=time.monotonic()
        while time.monotonic()-start<args.timeout and len(laps)<3:
            joy(True);spin(.02)
            m=latest['odom'];stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if stamp==last_stamp:continue
            last_stamp=stamp;xy=np.array([m.pose.pose.position.x,m.pose.pose.position.y]);v=m.twist.twist.linear.x
            control_stamps.append(stamp)
            fraction=np.clip(np.sum((xy-path)*seg,axis=1)/lengths**2,0,1)
            distances=np.linalg.norm(xy-path-fraction[:,None]*seg,axis=1);idx=int(np.argmin(distances));s=arc[idx]+fraction[idx]*lengths[idx]
            if lap_start is None:lap_start=stamp
            if previous is not None:progress+=(s-previous+arc[-1]/2)%arc[-1]-arc[-1]/2
            previous=s
            if progress>=(len(laps)+1)*arc[-1]:laps.append(stamp-lap_start);lap_start=stamp
            rows.append([stamp,xy[0],xy[1],v,float(distances[idx]),progress,latest['status'],*latest['command']])

            if latest['collision'] or distances[idx]>1.:raise RuntimeError('collision or tracking envelope')
        periods=np.diff(control_stamps)*1000.0
        result.update(laps_s=laps,failures=failures,collision=latest['collision'],collision_samples=latest['collision_count'],
                      rmse_m=float(np.sqrt(np.mean([r[4]**2 for r in rows]))),max_error_m=max(r[4] for r in rows),
                      max_speed_mps=max(r[3] for r in rows),progress_m=progress,
                      solver_max_ms=max(solver_ms, default=0.),
                      solver_p99_ms=float(np.percentile(solver_ms,99)) if solver_ms else 0.,
                      control_period_max_ms=max(periods, default=0.),
                      control_period_p99_ms=float(np.percentile(periods,99)) if len(periods) else 0.,
                      success=len(laps)==3 and not failures and not latest['collision'] and latest['collision_count']>0)
    except KeyboardInterrupt:
        result['error']='interrupted'
    except Exception as exc:result['error']=repr(exc)
    finally:
        # SIGINT may already invalidate rclpy. Cleanup must still terminate the
        # launch group and persist evidence even if sending the final stop fails.
        if node is not None:
            try:
                if rclpy.ok(): joy(False);spin(.3)
            except Exception as exc:
                result['cleanup_warning']=str(exc)
            try:
                node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
            except Exception as exc:
                result['cleanup_warning']=str(exc)
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        if 'laps_s' not in result:
            result.update(laps_s=laps,failures=failures,collision=latest['collision'],
                          collision_samples=latest['collision_count'],recording_complete=False)
        result.setdefault('solver_max_ms', max(solver_ms, default=0.))
        result.setdefault('solver_p99_ms', float(np.percentile(solver_ms,99)) if solver_ms else 0.)
        periods=np.diff(control_stamps)*1000.0
        result.setdefault('control_period_max_ms', max(periods, default=0.))
        result.setdefault('control_period_p99_ms', float(np.percentile(periods,99)) if len(periods) else 0.)
        log.close()
        with (args.output/'trajectory.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['stamp','x','y','speed','error','progress','status','command_speed','steer','acceleration']);writer.writerows(rows)
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
