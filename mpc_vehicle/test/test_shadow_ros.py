"""New safety adapter DDS test ONLY: no vehicle/simulator/driver is started.

Run explicitly in an isolated local ROS domain; not part of generic offline pytest.
"""
import json
import os
import time
from pathlib import Path
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('MPC_SHADOW_DDS_TEST') != 'YES', reason='explicit isolated DDS opt-in')


def test_shadow_gate_with_real_dds(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import Joy, LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    from mpc_vehicle.nodes import VehicleGate, command, RAW, MUX, HEALTH, FINAL
    folder = tmp_path/'11_16_3'; folder.mkdir()
    for name in ('global_waypoints.json', 'speed_scaling.yaml', 'ot_sectors.yaml', 'map.pgm'):
        (folder/name).write_text('{}')
    (folder/'11_16_3.yaml').write_text('image: map.pgm\n')
    p = json.loads((Path(__file__).parents[1]/'config/vehicle.example.json').read_text())
    p['map_dir'] = str(folder)
    for name in ('vehicle_config', 'static_transforms'):
        f=tmp_path/(name+'.yaml'); f.write_text('{}'); p[name]=str(f)
    profile=tmp_path/'profile.json'; profile.write_text(json.dumps(p))
    rclpy.init(args=['--ros-args', '-p', 'profile:='+str(profile)])
    executor=SingleThreadedExecutor()
    nodes=[]
    def make(name):
        node=Node(name); nodes.append(node); executor.add_node(node); return node
    sensors=make('test_sensors'); joynode=make('joy_node'); safety=make('vehicle_safety')
    control=make('mpc_vehicle_controller'); mux=make('mpc_vehicle_mux')
    gate=VehicleGate(); nodes.append(gate); executor.add_node(gate)
    state={'buttons': [0]*6, 'joy': True, 'scan': True, 'clear': True}
    jp=joynode.create_publisher(Joy, '/joy', 1)
    sp=sensors.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    pp=sensors.create_publisher(Odometry, '/car_state/odom', 1)
    op=sensors.create_publisher(Odometry, '/vesc/odom', 1)
    ep=safety.create_publisher(Bool, '/mpc_vehicle/estop_clear', 1)
    rp=control.create_publisher(AckermannDriveStamped, RAW, 1)
    hp=control.create_publisher(Bool, HEALTH, 1)
    mp=mux.create_publisher(AckermannDriveStamped, MUX, 1)
    def send():
        stamp=sensors.get_clock().now().to_msg()
        j=Joy(); j.header.stamp=stamp; j.axes=[0., 1., 0., 0.]; j.buttons=state['buttons']
        if state['joy']: jp.publish(j)
        scan=LaserScan(); scan.header.stamp=stamp; scan.angle_min=-.5; scan.angle_increment=.02
        scan.range_min=.05; scan.range_max=10.; scan.ranges=[3.]*51
        if state['scan']: sp.publish(scan)
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='map'; odom.child_frame_id='base_link'
        odom.pose.pose.orientation.w=1.; pp.publish(odom); op.publish(odom)
        ep.publish(Bool(data=state['clear']))
        rp.publish(command(control, .2, 0.)); hp.publish(Bool(data=True))
        mp.publish(command(mux, .2, 0.))
    timer=sensors.create_timer(.02, send)
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: executor.spin_once(timeout_sec=.01)
    def arm():
        state['buttons']=[0]*6; pump(.8)
        client=joynode.create_client(Trigger, '/mpc_vehicle/arm')
        assert client.wait_for_service(timeout_sec=1.)
        future=client.call_async(Trigger.Request()); pump(.2)
        assert future.done() and future.result().success
        joynode.destroy_client(client)
    try:
        pump(3.)
        assert gate.graph_ok, gate.graph_reason
        assert sensors.count_publishers(FINAL)==0  # no real actuator topic even after arm
        arm(); state['buttons'][5]=1; pump(.3)
        assert gate.policy.speed>0 and gate.policy.armed
        # Loss of joystick latches stop despite a healthy repeating MPC command.
        state['joy']=False; pump(.4)
        assert gate.policy.speed==0 and not gate.policy.armed
        state['joy']=True; pump(.3)
        assert gate.policy.speed==0
        arm(); state['buttons'][4]=1; pump(.2)
        assert gate.policy.speed>0
        state['clear']=False; pump(.1)
        assert gate.policy.speed==0 and not gate.policy.armed
        state['clear']=True
        arm(); state['buttons'][5]=1; pump(.2)
        bad=make('unexpected_controller'); bp=bad.create_publisher(AckermannDriveStamped, RAW, 1)
        pump(.5)
        assert not gate.graph_ok and not gate.policy.armed and gate.policy.speed==0
        bad.destroy_publisher(bp); pump(.5)
        arm(); state['buttons'][5]=1; pump(.2)
        state['scan']=False; pump(.4)
        assert not gate.policy.armed and gate.policy.speed==0
        assert sensors.count_publishers(FINAL)==0
    finally:
        for node in reversed(nodes): executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.shutdown()
