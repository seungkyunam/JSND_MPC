from setuptools import setup, find_packages
from glob import glob
setup(name='mpc_vehicle', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/mpc_vehicle']),
                  ('share/mpc_vehicle', ['package.xml']),
                  ('share/mpc_vehicle/launch', glob('launch/*launch.py')),
                  ('share/mpc_vehicle/config', glob('config/*.json') + glob('config/*.xml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='JSND MPC', maintainer_email='maintainer@example.invalid',
      description='Fail-closed low-speed vehicle commissioning for lateral MPC', license='MIT',
      entry_points={'console_scripts': ['mpc_vehicle_controller = mpc_vehicle.nodes:controller_main',
                                       'mpc_vehicle_gate = mpc_vehicle.nodes:gate_main',
                                       'mpc_vehicle_waypoints = mpc_vehicle.nodes:waypoints_main',
                                       'mpc_vehicle_estop = mpc_vehicle.nodes:estop_main']})
