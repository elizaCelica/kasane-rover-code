#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/eliza/osr_ws/install/setup.bash
exec ros2 launch osr_bringup osr_launch.py
