from pathlib import Path
import importlib.util
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
STACK = ROOT / 'jsnd_int/racing_stack'


def load_stage_module():
    path = ROOT/'jsnd_int/run/vehicle_final_stage.py'
    spec = importlib.util.spec_from_file_location('vehicle_final_stage', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_final_profile_keeps_c16bced_mpc_values_without_speed_tuning():
    params = yaml.safe_load((STACK/'stack_master/config/mpc_profile_obstacle_1s.yaml').read_text()) \
        ['controller_manager']['ros__parameters']
    assert 'mpc_max_speed_mps' not in params
    assert params == {
        'mpc_speed_source': 'integrated', 'mpc_horizon': 10,
        'mpc_preview_dt': 0.10888888888888888, 'mpc_preview_grid_power': 1.5,
        'mpc_time_scaled_objective': True, 'mpc_time_weighted_accel_cost': False,
        'mpc_q_position': 20.0, 'mpc_q_speed': 20.0, 'mpc_r_accel': 0.05,
        'mpc_braking_recovery_max_mps': 1.0, 'mpc_lateral_plan_fraction': 0.85,
        'mpc_brake_lead_s': 0.08, 'mpc_steer_preview_s': 0.03,
        'mpc_cmd_accel_lead_mps': 1.2,
    }


def test_launch_load_order_and_fixed_budget_are_fail_closed():
    launch = STACK/'stack_master/launch/race.launch.xml'
    ET.parse(launch)
    text = launch.read_text()
    speed = 'config/mpc_profile_$(var mpc_profile).yaml'
    final = 'config/mpc_profile_obstacle_1s.yaml'
    assert text.index(speed) < text.index(final)
    assert '<param name="mpc_deadline_s" value="0.020"' in text
    assert '<param name="mpc_solver_time_limit_s" value="0.020"' in text
    assert '<param name="mpc_final_vehicle_config" value="true"' in text
    validator = (STACK/'stack_master/launch/validate_final_vehicle.launch.py').read_text()
    for required in ("'mpc_rate_hz': '50'", "'mpc_speed_source': 'integrated'",
                     "'mpc_heading_robust': 'false'", "'mpc_history_prediction': 'true'"):
        assert required in validator


def test_runtime_wiring_retains_response_and_global_successor_and_records_required_data():
    manager = (STACK/'controller/controller/controller_manager.py').read_text()
    integrated = (STACK/'controller/controller/mpc/integrated_mpc.py').read_text()
    controller = (STACK/'controller/controller/combined/src/Controller.py').read_text()
    for field in ('vesc_speed_command_mps', 'requested_accel_mps2',
                  'measured_accel_mps2', 'solver_ms', 'status'):
        assert field in manager
    assert 'set_global_path' in integrated and '_append_global_successor' in integrated
    assert 'speed_response_tau_s' in integrated
    assert 'command = measured + cfg.speed_response_tau_s * accel' in controller
    recorder = (ROOT/'jsnd_int/run/step45_rec.sh').read_text()
    for topic in ('/controller/mpc_diagnostics', '/controller/mpc_predicted_path',
                  '/vesc/ackermann_cmd', '/vesc/odom', '/tracking/obstacles',
                  '/simple_mux/estop_state'):
        assert topic in recorder


def test_operator_scripts_offer_only_pp_or_fixed_mpc():
    launcher = (ROOT/'jsnd_int/run/step45_run.sh').read_text()
    assert 'pp|mpc)' in launcher
    assert 'mpc_rate_hz:=50' in launcher
    assert 'mpc_speed_source:=integrated' in launcher
    assert 'mpc_heading_robust:=false' in launcher
    assert 'mpc_history_prediction:=true' in launcher


def test_final_stage_backup_and_restore_are_checksum_guarded(tmp_path):
    stage = load_stage_module()
    target, backup = tmp_path/'racing_stack', tmp_path/'backup'
    target.mkdir()
    for name in stage.FILES:
        path = target/name
        path.parent.mkdir(parents=True, exist_ok=True)
        if name not in stage.NEW_FILES_ALLOWED:
            path.write_text('previous clean checkout\n')
    subprocess.run(['git', 'init', '-q'], cwd=target, check=True)
    subprocess.run(['git', 'config', 'user.email', 'test@example.invalid'], cwd=target, check=True)
    subprocess.run(['git', 'config', 'user.name', 'test'], cwd=target, check=True)
    subprocess.run(['git', 'add', '.'], cwd=target, check=True)
    subprocess.run(['git', 'commit', '-qm', 'baseline'], cwd=target, check=True)
    stage.apply(target, backup)
    assert (target/'stack_master/launch/validate_final_vehicle.launch.py').exists()
    stage.restore(backup)
    assert not (target/'stack_master/launch/validate_final_vehicle.launch.py').exists()
    assert not (target/'stack_master/config/mpc_profile_obstacle_1s.yaml').exists()
    assert (target/stage.FILES[0]).read_text() == 'previous clean checkout\n'
