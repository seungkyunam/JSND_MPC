import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


def invoke(*args, env=None, input_text=None):
    return subprocess.run(args, text=True, capture_output=True, env=env, input=input_text, timeout=60)


def test_scan_rejects_symlink_inside_package(tmp_path):
    package = tmp_path / 'controller'
    package.mkdir()
    (package / 'package.xml').write_text('<package><name>controller</name></package>')
    (package / 'escape').symlink_to(tmp_path, target_is_directory=True)
    result = invoke('python3', str(ROOT / 'mpc_deployment/inspect_sources.py'), 'scan', str(tmp_path))
    assert result.returncode != 0
    assert 'symlink directory' in result.stderr


@pytest.mark.parametrize('environment', [{}, {'SIM_ONLY_ISOLATED': 'YES', 'ROS_DOMAIN_ID': '0'}])
def test_run_rejects_unisolated_environment(tmp_path, environment):
    env = {**os.environ, **environment}
    if 'SIM_ONLY_ISOLATED' not in environment:
        env.pop('SIM_ONLY_ISOLATED', None)
    result = invoke('bash', str(ROOT / 'run_mpc.sh'), str(tmp_path), 'test0', 'mpc', env=env)
    assert result.returncode != 0
    assert 'Simulation only' in result.stderr


def test_deploy_preview_and_apply_only_touch_new_stage(tmp_path):
    if not Path('/opt/ros/humble/setup.bash').is_file():
        pytest.skip('Humble is not installed')
    workspace = tmp_path / 'vehicle-ws'
    source = workspace / 'src' / 'controller'
    source.mkdir(parents=True)
    (source / 'package.xml').write_text('<package><name>controller</name></package>')
    sentinel = source / 'sentinel.txt'
    sentinel.write_text('unchanged')
    install = workspace / 'install'
    install.mkdir()
    (install / 'setup.bash').write_text('source /opt/ros/humble/setup.bash\n')
    home = tmp_path / 'home'
    home.mkdir()
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    ssh = bin_dir / 'ssh'
    ssh.write_text('#!/bin/bash\nshift\nexec bash -c "$*"\n')
    ssh.chmod(0o755)
    env = {**os.environ, 'PATH': str(bin_dir) + os.pathsep + os.environ['PATH'], 'HOME': str(home)}
    args = ('bash', str(ROOT / 'deploy_mpc.sh'), 'vehicle.local', 'driver', str(workspace))
    preview = invoke(*args, env=env)
    assert preview.returncode == 0, preview.stderr
    assert 'ONLY IN VEHICLE SOURCE' in preview.stdout and 'sentinel.txt' in preview.stdout
    assert not list(home.iterdir())
    cancelled = invoke(*args, '--apply', env=env, input_text='no\n')
    assert cancelled.returncode != 0 and not list(home.iterdir())
    applied = invoke(*args, '--apply', env=env, input_text='STAGE\n')
    assert applied.returncode == 0, applied.stderr
    stages = list(home.glob('mpc_ws'))
    assert len(stages) == 1
    assert (stages[0] / 'stack_master/launch/validate_controller.launch.py').is_file()
    assert (stages[0] / 'controller/controller/mpc/lateral_mpc.py').is_file()
    assert (stages[0] / 'run_mpc.sh').is_file()
    assert sentinel.read_text() == 'unchanged'
    repeated = invoke(*args, '--apply', env=env, input_text='STAGE\n')
    assert repeated.returncode != 0
    assert 'Refusing to overwrite' in repeated.stderr
    assert sentinel.read_text() == 'unchanged'


def test_explicit_distro_mismatch_is_rejected(tmp_path):
    if not Path('/opt/ros/humble/setup.bash').is_file():
        pytest.skip('Humble is not installed')
    install = tmp_path / 'install'
    install.mkdir()
    (install / 'setup.bash').write_text('source /opt/ros/humble/setup.bash\n')
    result = invoke('bash', '-c', 'source "$1"; mpc_detect_distro "$2" jazzy',
                    '_', str(ROOT / 'mpc_deployment/ros_distro.bash'), str(tmp_path))
    assert result.returncode != 0
    assert 'differs from underlay' in result.stderr


@pytest.mark.parametrize('variable', ['ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE'])
def test_run_rejects_custom_discovery(tmp_path, variable):
    env = {**os.environ, 'SIM_ONLY_ISOLATED': 'YES', 'ROS_DOMAIN_ID': '213', variable: 'unsafe'}
    result = invoke('bash', str(ROOT / 'run_mpc.sh'), str(tmp_path), 'test0', env=env)
    assert result.returncode != 0
    assert 'Refusing custom DDS' in result.stderr


def probe_environment():
    import runpy
    return runpy.run_path(str(ROOT / 'mpc_deployment/sim_probe.py'))['launch_environment']


@pytest.mark.parametrize('distro,setting', [
    ('humble', {'ROS_LOCALHOST_ONLY': '1'}),
    ('jazzy', {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}),
])
def test_probe_discovery_matches_launcher(distro, setting):
    original = dict(SIM_ONLY_ISOLATED='YES', ROS_DOMAIN_ID='213', ROS_DISTRO=distro, **setting)
    child = probe_environment()(original)
    assert all(original[key] == value for key, value in setting.items())
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in child
    assert 'ROS_LOCALHOST_ONLY' not in child
    assert child['ROS_DISTRO'] == distro and child['ROS_DOMAIN_ID'] == '213'


@pytest.mark.parametrize('override', [
    {'ROS_STATIC_PEERS': 'remote'}, {'ROS_DISCOVERY_SERVER': 'remote'},
    {'CYCLONEDDS_URI': 'custom.xml'}, {'ROS_DOMAIN_ID': '0'},
    {'ROS_DOMAIN_ID': '233'}, {'SIM_ONLY_ISOLATED': 'NO'},
    {'ROS_DISTRO': 'humble'}, {'ROS_LOCALHOST_ONLY': '1'},
    {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'},
])
def test_probe_rejects_unsafe_or_mismatched_discovery(override):
    environment = dict(SIM_ONLY_ISOLATED='YES', ROS_DOMAIN_ID='213',
                       ROS_DISTRO='jazzy', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    with pytest.raises(ValueError):
        probe_environment()({**environment, **override})
