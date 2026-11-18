import json
from pathlib import Path
import pytest
from mpc_vehicle import core


def test_drive_report_bound_to_map_config_and_code(tmp_path, monkeypatch):
    p = json.loads((Path(__file__).parents[1]/'config/vehicle.example.json').read_text())
    folder=tmp_path/'11_16_3'; folder.mkdir()
    for name in ('global_waypoints.json', 'speed_scaling.yaml', 'ot_sectors.yaml', 'map.pgm'):
        (folder/name).write_text('{}')
    (folder/'11_16_3.yaml').write_text('image: map.pgm\n')
    p.update(map_dir=str(folder), vehicle_id='test-fixture', geometry_confirmed=True,
             bench_authorized=True, independent_estop_verified=True,
             actuator_watchdog_verified=True, watchdog_timeout_s=.2)
    vehicle=tmp_path/'car.yaml'; vehicle.write_text('''
/**:
  ros__parameters: {wheelbase: 0.324}
vesc/**:
  ros__parameters:
    steering_angle_to_servo_gain: -1.0
    steering_angle_to_servo_offset: 0.5
    servo_min: 0.0
    servo_max: 1.0
    speed_to_erpm_gain: 4614.0
    speed_to_erpm_offset: 0.0
    speed_min: -23000.0
    speed_max: 23000.0
    use_servo_cmd_to_calc_angular_velocity: true
''')
    p['vehicle_config']=str(vehicle)
    for key in ('static_transforms', 'hardware_verification_record'):
        f=tmp_path/key; f.write_text('unit test only'); p[key]=str(f)
    profile=tmp_path/'profile.json'; report=tmp_path/'approval.json'
    p['drive_approval']=str(report); profile.write_text(json.dumps(p))
    monkeypatch.setattr(core, 'software_id', lambda: 'test-only-code-id')
    report.write_text(json.dumps({'binding':core.approval_binding(p), 'operator':'test', 'date':'test',
                                 'checks':{k:True for k in core.CHECKS}}))
    assert core.load_profile(profile, 'drive')['vehicle_id']=='test-fixture'
    (folder/'global_waypoints.json').write_text('{"changed":true}')
    with pytest.raises(ValueError, match='stale'): core.load_profile(profile, 'drive')
    (folder/'global_waypoints.json').write_text('{}')
    monkeypatch.setattr(core, 'software_id', lambda: 'changed-code')
    with pytest.raises(ValueError, match='stale'): core.load_profile(profile, 'drive')


def test_launch_default_is_shadow_and_no_original_race_include():
    # Runtime DDS test separately proves that shadow has no final-topic publisher.
    import ast
    source=(Path(__file__).parents[1]/'launch/vehicle_mpc.launch.py').read_text()
    tree=ast.parse(source)
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
    assert any(isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument'
               and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value=='mode'
               and any(k.arg=='default_value' and k.value.value=='shadow' for k in n.keywords)
               for n in calls)
    assert 'IncludeLaunchDescription' not in source
