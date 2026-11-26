"""PF-only commissioning stack. Never includes race/low_level or their PP paths."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from mpc_vehicle.preflight import check
from mpc_vehicle.core import map_yaml_path


def configure(context):
    profile = LaunchConfiguration('profile').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    confirm = LaunchConfiguration('confirm').perform(context)
    map_dir = LaunchConfiguration('map_dir').perform(context)
    p = check(profile, mode, confirm, map_dir or None)
    node_params = {'profile': profile, 'mode': mode, 'map_dir': map_dir, 'use_sim_time': False}
    share = Path(get_package_share_directory('stack_master'))
    config = share/'config'
    vehicle = p['vehicle_config']
    nodes = []
    if mode != 'shadow':
        actuator_remaps = [('commands/'+s, '/mpc_vehicle/actuator/'+s)
                           for s in ('motor/speed', 'motor/brake', 'servo/position',
                                     'motor/current', 'motor/duty_cycle', 'motor/position')]
        nodes += [
            Node(package='vesc_driver', executable='vesc_driver_node', name='vesc_driver_node',
                 namespace='vesc', parameters=[vehicle], remappings=actuator_remaps, output='screen'),
            Node(package='vesc_ackermann', executable='ackermann_to_vesc_node', name='ackermann_to_vesc_node',
                 namespace='vesc', parameters=[vehicle], remappings=actuator_remaps, output='screen'),
            Node(package='vesc_ackermann', executable='vesc_to_odom_node', name='vesc_to_odom_node',
                 namespace='vesc', parameters=[vehicle, {'publish_tf': True}],
                 output='screen'),
            Node(package='urg_node', executable='urg_node_driver', name='urg_node', parameters=[vehicle], output='screen'),
            Node(package='stack_master', executable='static_tf_publisher.py', name='static_tf_publisher',
                 parameters=[p['static_transforms']], output='screen'),
            Node(package='stack_master', executable='imu_bias_corrector_node.py', name='imu_bias_corrector',
                 parameters=[vehicle], output='screen'),
            Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[{
                 'yaml_filename': str(map_yaml_path(Path(p['map_dir']))), 'frame_id': 'map',
                 'topic_name': 'map', 'use_sim_time': False}], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
                 parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}], output='screen'),
            Node(package='particle_filter', executable='particle_filter', name='particle_filter',
                 parameters=[str(config/'pf.yaml'), vehicle],
                 remappings=[('/clicked_point', '/pf_global_initialize')], output='screen'),
            Node(package='robot_localization', executable='ekf_node', name='ekf_localization',
                 parameters=[str(config/'ekf_pf.yaml')], remappings=[('odometry/filtered', '/car_state/odom')], output='screen'),
            # Race stack is stopped here, so the launch-selected map supplies /global_waypoints.
            Node(package='mpc_vehicle', executable='mpc_vehicle_waypoints', name='mpc_vehicle_waypoints',
                 parameters=[node_params], output='screen'),
        ]
    nodes += [
        Node(package='stack_master', executable='simple_mux_node.py', name='mpc_vehicle_mux',
             parameters=[{'in_topic': '/mpc_vehicle/raw_command', 'out_topic': '/mpc_vehicle/mux_command',
                          'joy_topic': '/joy', 'keyboard_joy_topic': '/mpc_vehicle/disabled_keyboard',
                          'ego_control_topic': '/mpc_vehicle/disabled_keyboard_select',
                          'joy_max_speed': float(p['speed_limit_mps']),
                          'joy_max_steer': float(p['max_steer_rad']), 'use_keyboard': False,
                          'joy_freshness_threshold': .2, 'use_estop': True, 'sim': False}], output='screen'),
        Node(package='mpc_vehicle', executable='mpc_vehicle_controller', name='mpc_vehicle_controller',
             parameters=[node_params], output='screen'),
        Node(package='mpc_vehicle', executable='mpc_vehicle_estop', name='mpc_vehicle_estop',
             parameters=[node_params], output='screen'),
        Node(package='mpc_vehicle', executable='mpc_vehicle_gate', name='mpc_vehicle_gate',
             parameters=[node_params], output='screen')]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='vehicle stack child exited; independent watchdog required'))]))
                for node in nodes]
    # Install exit handlers before starting any process.
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile'), DeclareLaunchArgument('mode', default_value='shadow'),
        DeclareLaunchArgument('confirm', default_value=''),
        # Empty for shadow (follows the running stack's /global_waypoints);
        # bench/drive: the map folder their own localization and waypoint source use.
        DeclareLaunchArgument('map_dir', default_value=''),
        OpaqueFunction(function=configure)])
