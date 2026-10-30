# ROS packages for JPL Open Source Rover Gazebo Simulation

> [!NOTE]
> This package isn't compiled with colcon by default. If you want to use this, remove the [COLCON_IGNORE file](./COLCON_IGNORE)

## Overview
Visualization and Gazebo Classic simulation for the OSR.

- `rviz.launch.py`: view the rover URDF in RViz
- `empty_world.launch.py`: spawn the rover in Gazebo and drive it with the **same** kinematics as the hardware stack

### Control architecture

Simulation no longer reimplements rocker-bogie math in C++. Instead:

1. `osr_control/rover` consumes `/cmd_vel` and publishes `/cmd_drive` + `/cmd_corner` using shared `osr_control.kinematics` and dimensions from `osr_bringup/config/osr_params.yaml`
2. `gazebo_command_adapter.py` bridges those commands to the Gazebo `wheel_controller` and `servo_controller`

Changing `d1`–`d4` / `wheel_radius` in `osr_params.yaml` therefore affects hardware and simulation the same way.

## Dependencies

This package still uses **Gazebo Classic** (`gazebo_ros` / `gazebo_ros2_control`), which is not supported on ROS 2 Jazzy. Use **ROS 2 Humble** + Gazebo Classic 11 until migrated to Gazebo Harmonic / `ros_gz`.

### Linux (simulation)
- **Operating System**: Ubuntu 22.04 LTS (Jammy)
- **ROS Distribution**: Humble
- **Gazebo**: Classic 11.x
- **Also required**: `osr_control`, `osr_interfaces`, `osr_bringup` (built in the same workspace)

## ROS Package Installation

```bash
sudo apt install python3-colcon-common-extensions
sudo apt-get install ros-humble-rviz2
sudo apt-get install ros-humble-controller-manager
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-joint-state-publisher
sudo apt-get install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-trajectory-msgs
sudo apt-get install ros-humble-velocity-controllers
sudo apt-get install ros-humble-joint-trajectory-controller
sudo apt-get install ros-humble-gazebo-ros2-control-demos
```

## Installation

Build from the ROS workspace root (so `osr_control` / `osr_interfaces` / `osr_bringup` are available):

```bash
source /opt/ros/humble/setup.bash
cd ~/osr_ws   # workspace that contains this repo's ROS/ packages
# remove COLCON_IGNORE under osr_gazebo first
colcon build --packages-select osr_interfaces osr_control osr_bringup osr_gazebo
source install/setup.bash
```

## Visualisation

```bash
ros2 launch osr_gazebo rviz.launch.py
```

## Simulation

```bash
ros2 launch osr_gazebo empty_world.launch.py
```

Drive with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> [!NOTE]
> Rotate-in-place on the hardware stack uses `Twist.angular.y` (joystick pitch). `teleop_twist_keyboard` only commands `angular.z`, so in-place rotation needs a publisher that sets `angular.y` with zero `linear.x`, or the intuitive/joy setup from `osr_bringup`.

## Follow-ups
- Bridge Gazebo `/joint_states` into `/drive_state` and `/corner_state` so `rover` odometry can run in sim
- Migrate from Gazebo Classic to Harmonic / `ros_gz` for Jazzy

## The method to convert from Onshape to URDF

- The object file for the rover is available within Onshape.
- This object file can be disassembled into its individual components, such as rocker bogie1,2,3, and box, etc..
- However, directly using it as a URDF after converting it to an STL will result in significant CPU and GPU usage in RViz or Gazebo due to the file size issue.
- Therefore, the process of reducing the file size of the STL using MeshLab was carried out.
- The package provided at https://github.com/gstavrinos/calc-inertia was then used to define the inertial properties.
