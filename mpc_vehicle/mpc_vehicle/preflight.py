"""Checks before creating ANY hardware node. Called by launch, not only shell wrapper."""
import time
import os
import json
import fcntl
from pathlib import Path
from .core import load_profile

DDS_PROFILE = 'cyclonedds_vehicle_localhost.xml'


def packaged_dds_uri():
    """URI of the loopback-only CycloneDDS profile installed with this package."""
    from ament_index_python.packages import get_package_share_directory
    return 'file://' + str(Path(get_package_share_directory('mpc_vehicle')) / 'config' / DDS_PROFILE)


def check_dds_environment():
    """Only the packaged loopback profile (or none) may configure DDS."""
    for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_STATIC_PEERS'):
        if os.environ.get(key): raise ValueError('custom DDS configuration refused: '+key)
    uri = os.environ.get('CYCLONEDDS_URI')
    if uri:
        if uri != packaged_dds_uri():
            raise ValueError('custom DDS configuration refused: CYCLONEDDS_URI')
        if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
            raise ValueError('packaged CycloneDDS profile requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')


def check(profile, mode, confirmation, map_dir=None):
    p = load_profile(profile, mode, map_dir)
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise ValueError('vehicle launch requires ROS 2 Jazzy')
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError('explicit ROS_DOMAIN_ID 0..232 required')
    if (os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('ROS_LOCALHOST_ONLY')):
        raise ValueError('Jazzy LOCALHOST discovery required; unset ROS_LOCALHOST_ONLY')
    check_dds_environment()
    if mode == 'shadow': return p
    expected = 'WHEELS_LIFTED' if mode == 'bench' else 'CONTROLLED_LOW_SPEED_TEST'
    if confirmation != expected:
        raise ValueError('explicit physical-condition confirmation required: '+expected)
    # Same-host collision check. Gate holds this lock throughout its lifetime.
    with open(f'/tmp/jsnd_mpc_vehicle_{os.getuid()}.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('mpc_vehicle_preflight')
    try:
        end = time.monotonic()+3.
        while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.1)
        forbidden = ('/vesc/', '/car_state/', '/mpc_vehicle/actuator/')
        for topic, types in node.get_topic_names_and_types():
            if (topic.startswith(forbidden) or topic in ('/scan', '/map')) and node.count_publishers(topic):
                raise ValueError('existing hardware/control graph must be stopped: '+topic)
        for name, ns in node.get_node_names_and_namespaces():
            if name in ('controller_manager', 'simple_mux', 'mpc_vehicle_gate', 'mpc_vehicle_mux',
                        'particle_filter', 'ekf_localization', 'vesc_driver_node'):
                raise ValueError('conflicting node: '+ns+'/'+name)
    finally:
        node.destroy_node(); rclpy.shutdown()
    # Firmware supervision is independent; see required physical verification record.
    return p
