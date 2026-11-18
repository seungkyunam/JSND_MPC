#!/usr/bin/env python3
"""Observe a stationary, explicitly isolated simulation; publishes no commands."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

def launch_environment(environment):
    """Validate observer discovery and let run_mpc set the child's discovery."""
    env = dict(environment)
    distro = env.get('ROS_DISTRO')
    domain = env.get('ROS_DOMAIN_ID', '')
    if (env.get('SIM_ONLY_ISOLATED') != 'YES' or not domain.isascii()
            or not domain.isdigit() or not 1 <= len(domain) <= 3
            or not 1 <= int(domain) <= 232):
        raise ValueError('requires explicitly isolated sim-only environment and nonzero domain')
    for name in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI',
                 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                 'ROS_STATIC_PEERS'):
        if env.get(name):
            raise ValueError(f'refusing custom DDS configuration: {name}')
    if distro == 'humble':
        valid = env.get('ROS_LOCALHOST_ONLY') == '1' and not env.get('ROS_AUTOMATIC_DISCOVERY_RANGE')
    elif distro == 'jazzy':
        valid = env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST' and not env.get('ROS_LOCALHOST_ONLY')
    else:
        raise ValueError('source a matching Humble or Jazzy environment')
    if not valid:
        raise ValueError('observer discovery setting does not match ROS distribution')
    env.pop('ROS_AUTOMATIC_DISCOVERY_RANGE', None)
    env.pop('ROS_LOCALHOST_ONLY', None)
    return env


def parameters(node, target, names):
    client = node.create_client(GetParameters, f'/{target}/get_parameters')
    if not client.wait_for_service(timeout_sec=4):
        raise RuntimeError(f'parameter service unavailable: {target}')
    request = GetParameters.Request()
    request.names = names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=4)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'parameter request failed: {target}')
    return {name: value.bool_value if value.type == 1 else value.string_value
            for name, value in zip(names, future.result().values)}


def main():
    if len(sys.argv) != 4:
        raise ValueError('usage: sim_probe.py BUILD_DIR MAP NEW_OUTPUT_DIR')
    child_env = launch_environment(os.environ)
    global rclpy, GetParameters
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from f110_msgs.msg import WpntArray
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import Float32
    build, map_name, output_name = sys.argv[1:]
    output = Path(output_name)
    output.mkdir(parents=True, exist_ok=False)
    stage = Path(__file__).resolve().parents[1]
    result = {'sim_only': True, 'map': map_name, 'ros_distro': os.environ['ROS_DISTRO'], 'checks': {}, 'success': False}
    process = None
    node = None
    initialized = False
    try:
        with (output / 'launch.log').open('w', encoding='utf-8') as log:
            process = subprocess.Popen(
                ['bash', str(stage / 'run_mpc.sh'), build, map_name, 'mpc', '--ros-distro', os.environ['ROS_DISTRO']],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=child_env,
            )
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline and process.poll() is None:
                if 'Simulation only. ROS domain checks' in (output / 'launch.log').read_text(errors='replace'):
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError('simulation launch did not pass its isolation preflight')

            rclpy.init()
            initialized = True
            node = rclpy.create_node('mpc_deployment_observer')
            observed = {'waypoints': 0, 'commands': 0, 'solve_times': 0}
            node.create_subscription(WpntArray, '/global_waypoints',
                                     lambda msg: observed.update(waypoints=len(msg.wpnts)), 10)
            node.create_subscription(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd',
                                     lambda msg: observed.update(commands=observed['commands'] + 1), 10)
            node.create_subscription(Float32, '/controller/mpc_elapsed_ms',
                                     lambda msg: observed.update(solve_times=observed['solve_times'] + 1), 10)
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.2)
                names = set(node.get_node_names())
                if {'bridge', 'controller_manager', 'simple_mux', 'global_republisher'} <= names and all(observed.values()):
                    break
            names = set(node.get_node_names())
            controller = parameters(node, 'controller_manager', ['sim', 'lateral_controller', 'drive_topic'])
            mux = parameters(node, 'simple_mux', ['joy_topic', 'in_topic', 'out_topic'])
            publisher = parameters(node, 'global_republisher', ['map'])
            drive_publishers = [item.node_name for item in node.get_publishers_info_by_topic('/vesc/high_level/ackermann_cmd')]
            drive_subscribers = [item.node_name for item in node.get_subscriptions_info_by_topic('/vesc/high_level/ackermann_cmd')]
            joy_subscribers = [item.node_name for item in node.get_subscriptions_info_by_topic('/joy')]
            result.update(controller=controller, mux=mux, global_republisher=publisher,
                          observed=observed, drive_publishers=drive_publishers,
                          drive_subscribers=drive_subscribers, joy_subscribers=joy_subscribers)
            result['checks'] = {
                'simulator_node': 'bridge' in names,
                'mpc_sim_controller': controller.get('sim') is True and controller.get('lateral_controller') == 'MPC',
                'selected_map': publisher.get('map') == map_name and observed['waypoints'] > 0,
                'controller_to_mux': 'controller_manager' in drive_publishers and 'simple_mux' in drive_subscribers,
                'joy_to_mux': 'simple_mux' in joy_subscribers,
                'command_observed': observed['commands'] > 0,
                'mpc_calculation_observed': observed['solve_times'] > 0,
                'mux_output_topic': mux.get('out_topic') == '/vesc/ackermann_cmd',
            }
            result['success'] = all(result['checks'].values())
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
