"""Checks before creating ANY hardware node. Called by launch, not only shell wrapper."""
import time
import os
import json
import fcntl
from pathlib import Path
from .core import load_profile


def check(profile, mode, confirmation):
    p = load_profile(profile, mode)
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise ValueError('vehicle launch requires ROS 2 Jazzy')
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError('explicit ROS_DOMAIN_ID 0..232 required')
    if (os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('ROS_LOCALHOST_ONLY')):
        raise ValueError('Jazzy LOCALHOST discovery required; unset ROS_LOCALHOST_ONLY')
    for key in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_STATIC_PEERS'):
        if os.environ.get(key): raise ValueError('custom DDS configuration refused: '+key)
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
