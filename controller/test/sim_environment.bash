#!/usr/bin/env bash
# Source only in an isolated simulation terminal. No writes to the underlay.
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/sk/JSND/.humble-test/install/local_setup.bash
export PYTHONPATH="/home/sk/JSND_MPC/race_utils/unicorn_gym/f1tenth_gym/gym:/home/sk/JSND_MPC/controller:/home/sk/JSND_MPC/.venv/lib/python3.10/site-packages:/home/sk/JSND/.humble-test/python:/home/sk/miniforge3/envs/jsnd-humble/lib/python3.10/site-packages:/home/sk/.local/lib/python3.10/site-packages:$PYTHONPATH:/usr/lib/python3/dist-packages"
export ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/home/sk/JSND_MPC/.humble-test/ros-log
export MPLCONFIGDIR=/home/sk/JSND_MPC/.humble-test/matplotlib
export NUMBA_CACHE_DIR=/home/sk/JSND_MPC/.humble-test/numba
export RAYCASTER_PATH=/home/sk/JSND/race_utils/raycaster
export RAYCASTER_DIR=/home/sk/JSND/race_utils/raycaster
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MPLBACKEND=Agg QT_QPA_PLATFORM=offscreen
export PYTHONNOUSERSITE=1
