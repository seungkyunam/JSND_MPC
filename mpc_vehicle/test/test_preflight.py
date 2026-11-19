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


PACKAGED = 'file:///opt/fixture/share/mpc_vehicle/config/cyclonedds_vehicle_localhost.xml'


@pytest.fixture
def packaged_dds(local_env, monkeypatch):
    monkeypatch.setattr(preflight, 'packaged_dds_uri', lambda: PACKAGED)
    monkeypatch.setenv('CYCLONEDDS_URI', PACKAGED)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')


def test_packaged_loopback_dds_profile_is_accepted(packaged_dds):
    assert preflight.check('fixture', 'shadow', '') == {'fixture': True}


@pytest.mark.parametrize('key,value', [('CYCLONEDDS_URI', 'file:///home/x/racing_stack/cyclonedds.xml'),
                                      ('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
                                      ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
                                      ('ROS_STATIC_PEERS', '192.168.0.2')])
def test_packaged_profile_does_not_open_other_dds_settings(packaged_dds, monkeypatch, key, value):
    monkeypatch.setenv(key, value)
    with pytest.raises(ValueError): preflight.check('fixture', 'shadow', '')


def test_packaged_profile_is_loopback_only():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    root = ET.parse(Path(__file__).parents[1]/'config'/preflight.DDS_PROFILE).getroot()
    ns = {'c': 'https://cdds.io/config'}
    interfaces = root.findall('.//c:Interfaces/c:NetworkInterface', ns)
    assert [i.attrib.get('address') for i in interfaces] == ['127.0.0.1']
    assert not any('name' in i.attrib for i in interfaces)
