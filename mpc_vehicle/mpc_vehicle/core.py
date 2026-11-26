"""ROS-independent commissioning policy. No actuator or PP code here."""
import hashlib
import json
import math
from pathlib import Path

CHECKS = ('shadow_no_actuator_publishers', 'physical_estop', 'driver_watchdog',
          'forward_speed_sign', 'left_right_steering_sign', 'manual_takeover',
          'joy_disconnect', 'command_loss', 'sensor_loss', 'duplicate_controller',
          'gate_process_killed', 'no_automatic_rearm', 'map_pose_alignment')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def software_id():
    h = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob('*.py')):
        h.update(path.name.encode()); h.update(path.read_bytes())
    # Includes the actual installed integrated MPC and original mux/vehicle assets.
    import controller.mpc.integrated_mpc as mpc
    for path in sorted(Path(mpc.__file__).parent.glob('*.py')):
        h.update(path.name.encode()); h.update(path.read_bytes())
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    roots = [Path(get_package_share_directory('mpc_vehicle')) / 'launch',
             Path(get_package_share_directory('stack_master')) / 'config',
             Path(get_package_prefix('stack_master')) / 'lib' / 'stack_master']
    for root in roots:
        for path in sorted(root.rglob('*')):
            if path.is_file() and path.suffix in ('.py', '.yaml'):
                h.update(str(path.relative_to(root)).encode()); h.update(path.read_bytes())
    return h.hexdigest()


def map_yaml_path(folder):
    """Resolve a map's YAML without tying commissioning to one map name."""
    folder = Path(folder)
    named = folder / f'{folder.name}.yaml'
    if named.is_file():
        return named
    import yaml
    candidates = []
    for path in sorted(folder.glob('*.yaml')):
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except Exception:
            continue
        if isinstance(data, dict) and isinstance(data.get('image'), str):
            candidates.append(path)
    if len(candidates) != 1:
        raise ValueError('map_dir must contain one map YAML with an image entry')
    return candidates[0]


def validate_map_dir(folder):
    """Map folder used by bench/drive localization; never stored in the vehicle profile."""
    folder = Path(folder)
    map_yaml = map_yaml_path(folder)
    for name in ('global_waypoints.json', 'speed_scaling.yaml', 'ot_sectors.yaml'):
        if not (folder/name).is_file(): raise ValueError(f'missing map file: {name}')
    import yaml
    map_data = yaml.safe_load(map_yaml.read_text())
    image = Path(map_data['image'])
    if image.is_absolute() or '..' in image.parts or not (folder/image).is_file():
        raise ValueError('map image must be copied inside map directory')
    return folder


def validate_raceline(xy, speeds, max_gap=.5):
    """Closed global raceline -> (xy[N,2], speeds[N]); raises ValueError if unusable."""
    import numpy as np
    xy = np.asarray(xy, dtype=float); speeds = np.asarray(speeds, dtype=float)
    if xy.ndim != 2 or xy.shape[1:] != (2,) or speeds.shape != (len(xy),):
        raise ValueError('malformed_raceline')
    if len(xy) >= 2 and np.linalg.norm(xy[0]-xy[-1]) < 1.e-6:
        xy, speeds = xy[:-1], speeds[:-1]
    if len(xy) < 20: raise ValueError('too_few_waypoints')
    if not np.isfinite(xy).all() or not np.isfinite(speeds).all() or np.any(speeds < 0):
        raise ValueError('nonfinite_or_negative_raceline')
    if np.max(np.linalg.norm(np.roll(xy, -1, axis=0)-xy, axis=1)) > max_gap:
        raise ValueError('raceline_not_closed_loop')
    return xy, speeds


STANDSTILL_NOISE_MPS = .02


def solver_speed(measured):
    """Measured forward speed for the solver (which rejects any negative speed).

    Only standstill sensor noise (EKF jitter of ~1e-4 m/s while stopped) is
    clamped to 0. Real reverse motion is still refused, never hidden.
    """
    measured = float(measured)
    if not math.isfinite(measured): raise ValueError('nonfinite speed')
    if measured < -STANDSTILL_NOISE_MPS: raise ValueError('reverse_motion')
    return max(0., measured)


def combine_speeds(xy, speeds, scaled=None):
    """Use the sector-scaled profile only to lower speeds, and only if it is the same line."""
    import numpy as np
    if scaled is None: return speeds
    scaled_xy, scaled_speeds = scaled
    if scaled_xy.shape != xy.shape or np.max(np.abs(scaled_xy-xy)) > 1.e-3: return speeds
    return np.minimum(speeds, scaled_speeds)


def approval_binding(p):
    normalized = dict(p); normalized['drive_approval'] = ''
    h = hashlib.sha256(json.dumps(normalized, sort_keys=True).encode())
    for directory in [Path(p['map_dir'])]:
        for path in sorted(directory.rglob('*')):
            if path.is_file():
                h.update(str(path.relative_to(directory)).encode()); h.update(path.read_bytes())
    for key in ('vehicle_config', 'static_transforms', 'hardware_verification_record'):
        h.update(Path(p[key]).read_bytes())
    h.update(software_id().encode())
    return h.hexdigest()


def load_profile(path, mode, map_dir=None):
    """Vehicle-only profile. Maps are never part of it:

    shadow follows the running stack's /global_waypoints; bench/drive take an
    explicit map_dir argument for their own localization and waypoint source.
    """
    if mode not in ('shadow', 'bench', 'drive'):
        raise ValueError('mode must be shadow, bench or drive')
    p = json.loads(Path(path).read_text())
    if 'map_dir' in p:
        raise ValueError('vehicle profile must not contain map_dir; pass the map at launch instead')
    for name, low, high in [('wheelbase', .1, 1.), ('max_steer_rad', .01, .3),
                          ('max_steer_rate_radps', .01, 1.), ('speed_limit_mps', .01, .5),
                          ('acceleration_mps2', .01, .5), ('deceleration_mps2', .01, 2.),
                          ('lateral_acceleration_mps2', .01, 2.)]:
        v = p[name]
        if type(v) not in (int, float) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(f'invalid commissioning limit: {name}')
    if not math.isfinite(p['pose_to_rear_axle_m']) or abs(p['pose_to_rear_axle_m']) > 1:
        raise ValueError('invalid signed rear axle offset')
    import yaml
    if mode == 'shadow':
        if map_dir:
            raise ValueError('shadow follows the running stack /global_waypoints; do not pass map_dir')
    else:
        if not map_dir:
            raise ValueError(f'{mode} requires an explicit map_dir argument')
        p['map_dir'] = str(validate_map_dir(map_dir))
    for key in ('vehicle_config', 'static_transforms'):
        if not Path(p[key]).is_file(): raise ValueError(f'missing {key}')
    for name in ('joy_source_node', 'estop_source_node'):
        if not isinstance(p[name], str) or not p[name].startswith('/') or p[name] == '/':
            raise ValueError(f'{name} must be an exact fully qualified node name')
    if mode != 'shadow':
        for key in ('geometry_confirmed', 'bench_authorized', 'independent_estop_verified',
                    'actuator_watchdog_verified'):
            if p[key] is not True: raise ValueError(f'hardware output blocked: {key}')
        if p['vehicle_id'] == 'UNVERIFIED' or not p['vehicle_id'].strip():
            raise ValueError('identify the actual vehicle')
        if not 0 < p['watchdog_timeout_s'] <= .25:
            raise ValueError('independent actuator watchdog <= 0.25 s required')
        vehicle = yaml.safe_load(Path(p['vehicle_config']).read_text())
        common = vehicle['/**']['ros__parameters']
        actuator = vehicle['vesc/**']['ros__parameters']
        if not math.isclose(float(common['wheelbase']), p['wheelbase'], abs_tol=1.e-6):
            raise ValueError('MPC and CAR wheelbase differ')
        gain = float(actuator['steering_angle_to_servo_gain'])
        offset = float(actuator['steering_angle_to_servo_offset'])
        values = [gain, offset, float(actuator['servo_min']), float(actuator['servo_max']),
                  float(actuator['speed_to_erpm_gain']), float(actuator['speed_to_erpm_offset']),
                  float(actuator['speed_min']), float(actuator['speed_max'])]
        if not all(math.isfinite(v) for v in values) or gain == 0 or values[4] <= 0:
            raise ValueError('invalid actuator conversion')
        for angle in (-p['max_steer_rad'], 0., p['max_steer_rad']):
            if not 0 <= values[2] <= offset+gain*angle <= values[3] <= 1:
                raise ValueError('steering envelope exceeds calibrated servo limits')
        for speed in (0., p['speed_limit_mps']):
            if not values[6] <= values[4]*speed+values[5] <= values[7]:
                raise ValueError('speed envelope exceeds ERPM limits')
        if actuator.get('use_servo_cmd_to_calc_angular_velocity') is not True:
            raise ValueError('this launch requires servo command odometry input')
        record = Path(p['hardware_verification_record'])
        if not record.is_file() or not record.read_text().strip():
            raise ValueError('hardware estop/watchdog verification record required')
        if mode == 'drive':
            report = json.loads(Path(p['drive_approval']).read_text())
            if (report.get('binding') != approval_binding(p)
                    or not report.get('operator') or not report.get('date')
                    or any(report.get('checks', {}).get(k) is not True for k in CHECKS)):
                raise ValueError('missing/stale wheels-up approval for this software/profile/map')
    return p


class GatePolicy:
    """Latched stop, explicit neutral re-arm, hold-to-run and bounded output.

    Uses monotonic receipt ages; the ROS adapter additionally checks header ages,
    graph identities, laser validity and MPC health. Stop is never ramped down.
    """
    def __init__(self, profile):
        self.p = profile
        self.armed = False
        self.engaged = False
        self.reason = 'startup_disarmed'
        self.speed = 0.
        self.steer = 0.
        self.last_tick = None
        self.neutral_since = None

    def stop(self, reason):
        self.armed = False
        self.reason = reason
        self.speed = 0.
        return 0., self.steer

    def arm(self, now, ready, manual, auto, emergency):
        if (not ready or emergency or manual or auto or self.neutral_since is None
                or now - self.neutral_since < .5):
            return False
        self.armed = True
        self.engaged = False
        self.reason = 'armed_idle'
        return True

    def tick(self, now, *, ready, manual, auto, emergency, command, auto_ok):
        dt = .02 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if not manual and not auto and ready and not emergency:
            if self.neutral_since is None: self.neutral_since = now
        else: self.neutral_since = None
        if not ready or emergency or not 0 < dt <= .1:
            return self.stop('emergency_or_stale_input_or_graph_or_loop')
        if not self.armed: return 0., self.steer
        if not manual and not auto:
            return self.stop('deadman_released') if self.engaged else (0., self.steer)
        self.engaged = True
        if not manual and auto and not auto_ok:
            return self.stop('mpc_not_healthy')
        if command is None:
            self.speed = 0.
            return 0., self.steer  # mux mode transition: wait for matching fresh sample
        speed, steer = command
        if not all(math.isfinite(v) for v in command): return self.stop('nonfinite_command')
        speed = max(0., min(self.p['speed_limit_mps'], speed))
        steer = max(-self.p['max_steer_rad'], min(self.p['max_steer_rad'], steer))
        # Immediate stop/deceleration, bounded positive acceleration and steer slew.
        self.speed = min(speed, self.speed + self.p['acceleration_mps2'] * dt)
        delta = self.p['max_steer_rate_radps'] * dt
        self.steer = max(self.steer-delta, min(self.steer+delta, steer))
        self.reason = 'manual' if manual else 'mpc'
        return self.speed, self.steer
