"""Endpoint arbitration adapter tests. No ROS context or hardware nodes."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from mpc_vehicle.nodes import VehicleGate, PREFIX, RAW, MUX, HEALTH, APPLIED, FINAL, ACTUATORS, DISABLED
from mpc_vehicle.core import GatePolicy


def endpoint(name):
    namespace, basename=name.rsplit('/', 1)
    return SimpleNamespace(node_namespace=namespace or '/', node_name=basename)


def graph():
    profile={'joy_source_node':'/joy_node', 'estop_source_node':'/vehicle_safety'}
    owners={RAW:'/mpc_vehicle_controller', HEALTH:'/mpc_vehicle_controller',
            MUX:'/mpc_vehicle_mux', APPLIED:'/mpc_vehicle_gate', FINAL:'/mpc_vehicle_gate',
            '/joy':'/joy_node', PREFIX+'/estop_clear':'/vehicle_safety',
            '/scan':'/urg_node', '/car_state/odom':'/ekf_localization', '/vesc/odom':'/vesc/vesc_to_odom_node'}
    for suffix in ACTUATORS: owners[PREFIX+'/actuator/'+suffix]='/vesc/ackermann_to_vesc_node'
    pubs={topic:[endpoint(name)] for topic,name in owners.items()}
    subs={FINAL:[endpoint('/vesc/ackermann_to_vesc_node')]}
    for suffix in ACTUATORS: subs[PREFIX+'/actuator/'+suffix]=[endpoint('/vesc/vesc_driver_node')]
    names=set(owners.values())|{'/vesc/vesc_driver_node'}
    fake=SimpleNamespace(p=profile, mode='bench', policy=GatePolicy({}),
        get_publishers_info_by_topic=lambda topic:pubs.get(topic,[]),
        get_subscriptions_info_by_topic=lambda topic:subs.get(topic,[]),
        get_node_names_and_namespaces=lambda:[(x.rsplit('/',1)[1],x.rsplit('/',1)[0] or '/') for x in names])
    fake.owned=lambda *a:VehicleGate.owned(fake,*a)
    return fake,pubs,subs


def test_normal_private_driver_topology_is_accepted():
    node,_,_=graph(); VehicleGate.inspect_graph(node)
    assert node.graph_ok, node.graph_reason


@pytest.mark.parametrize('topic', [RAW, MUX, FINAL, '/vesc/high_level/ackermann_cmd',
                                   PREFIX+'/actuator/motor/current', '/vesc/commands/motor/duty_cycle',
                                   PREFIX+'/disabled_keyboard_select'])
def test_foreign_publisher_latches_stop(topic):
    node,pubs,_=graph(); node.policy.armed=True
    pubs.setdefault(topic,[]).append(endpoint('/foreign_controller'))
    VehicleGate.inspect_graph(node)
    assert not node.graph_ok and not node.policy.armed


def test_unexpected_actuator_subscriber_rejected():
    node,_,subs=graph(); subs[FINAL].append(endpoint('/second_converter'))
    VehicleGate.inspect_graph(node)
    assert not node.graph_ok


def test_solver_failure_latches_even_if_health_recovers_before_timer_tick():
    node=SimpleNamespace(inputs={'joy':SimpleNamespace(buttons=[0,0,0,0,0,1])},
                         times={},policy=GatePolicy({}))
    node.policy.armed=True
    VehicleGate.receive(node,'health',SimpleNamespace(data=False))
    VehicleGate.receive(node,'health',SimpleNamespace(data=True))
    assert not node.policy.armed


def test_runtime_actuator_calibration_requires_restart():
    node=SimpleNamespace(config_ok=True,policy=GatePolicy({}))
    node.policy.armed=True
    VehicleGate.parameter_event(node,SimpleNamespace(node='/vesc/ackermann_to_vesc_node',
                                                    changed_parameters=[object()],deleted_parameters=[]))
    assert not node.config_ok and not node.policy.armed
