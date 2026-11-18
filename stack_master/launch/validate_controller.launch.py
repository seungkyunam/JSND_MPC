from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def validate_controller(context):
    controller = LaunchConfiguration('controller').perform(context)
    sim = LaunchConfiguration('sim').perform(context)
    if controller not in ('pp', 'mpc'):
        raise ValueError('controller must be pp or mpc')
    if controller == 'mpc' and sim != 'true':
        raise ValueError('MPC is experimental and requires sim:=true; real vehicle use is disabled')
    return []


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=validate_controller)])
