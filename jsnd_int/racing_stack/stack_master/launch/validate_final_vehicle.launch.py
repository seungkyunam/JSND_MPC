"""Fail closed when an MPC vehicle launch departs from the c16bced release."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


SAFE_PROFILES = ('lowspeed', '1p5', '2p25', '3p0', '4p0', '5p0')


def _validate(context):
    if LaunchConfiguration('controller').perform(context) != 'mpc':
        return []
    values = {
        'mpc_rate_hz': LaunchConfiguration('mpc_rate_hz').perform(context),
        'mpc_speed_source': LaunchConfiguration('mpc_speed_source').perform(context),
        'mpc_heading_robust': LaunchConfiguration('mpc_heading_robust').perform(context).lower(),
        'mpc_history_prediction': LaunchConfiguration('mpc_history_prediction').perform(context).lower(),
    }
    expected = {
        'mpc_rate_hz': '50',
        'mpc_speed_source': 'integrated',
        'mpc_heading_robust': 'false',
        'mpc_history_prediction': 'true',
    }
    wrong = [f'{key}={values[key]} (required {value})'
             for key, value in expected.items() if values[key] != value]
    profile = LaunchConfiguration('mpc_profile').perform(context)
    if profile not in SAFE_PROFILES:
        wrong.append(f'mpc_profile={profile} (allowed: {", ".join(SAFE_PROFILES)})')
    if wrong:
        raise ValueError('final MPC vehicle configuration rejected: ' + '; '.join(wrong))
    return []


def generate_launch_description():
    defaults = {
        'controller': 'pp', 'mpc_profile': 'lowspeed', 'mpc_rate_hz': '50',
        'mpc_speed_source': 'integrated', 'mpc_heading_robust': 'false',
        'mpc_history_prediction': 'true',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=_validate),
    ])
