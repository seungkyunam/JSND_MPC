import pytest
from mpc_vehicle import preflight


@pytest.fixture
def local_env(monkeypatch):
    monkeypatch.setattr(preflight, 'load_profile', lambda *_: {'fixture': True})
    for key in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY'):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('ROS_DISTRO','jazzy')
    monkeypatch.setenv('ROS_DOMAIN_ID','42')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')


def test_shadow_preflight_never_initializes_ros(local_env):
    assert preflight.check('fixture', 'shadow', '') == {'fixture': True}


@pytest.mark.parametrize('key,value', [('ROS_DISTRO','humble'), ('ROS_DOMAIN_ID','233'),
                                      ('ROS_AUTOMATIC_DISCOVERY_RANGE','SUBNET'),
                                      ('ROS_LOCALHOST_ONLY','1'), ('CYCLONEDDS_URI','custom')])
def test_direct_launch_rejects_wrong_environment_before_nodes(local_env, monkeypatch, key, value):
    monkeypatch.setenv(key,value)
    with pytest.raises(ValueError): preflight.check('fixture','bench','WHEELS_LIFTED')


def test_hardware_requires_physical_confirmation_before_ros_init(local_env):
    with pytest.raises(ValueError,match='physical-condition'):
        preflight.check('fixture','bench','')
