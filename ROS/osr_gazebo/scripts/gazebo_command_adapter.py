#!/usr/bin/env python3
"""Bridge CommandDrive / CommandCorner from rover.py to Gazebo ros2_control topics.

Hardware CommandDrive uses opposite signs on the right-side motors (mounting).
Gazebo joint axes expect the same rotational sense for forward motion on all
wheels, so right-side velocities are negated again here.
Corner commands use the motor frame from rover.py (z down); Gazebo steering
joints use the opposite sign convention.

Wheel speeds are ramped toward the commanded value at a limited acceleration,
like the RoboClaws do on the real rover (drive_acceleration_factor). Without
the ramp, the velocity controller changes wheel speed in a single physics step
and the rover pitches and hops when it stops or reverses.

In the other direction, the simulated joint states (/joint_states) are
republished as drive_state and corner_state, with the names and sign
conventions the RoboClaw and servo nodes use on the real rover, so rover.py's
wheel odometry runs unchanged in simulation.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from osr_interfaces.msg import CommandCorner, CommandDrive


class GazeboCommandAdapter(Node):
    # Matches wheel_controller joints order in controller_velocity.yaml
    WHEEL_JOINT_ORDER = (
        'middle_wheel_joint_right',
        'middle_wheel_joint_left',
        'front_wheel_joint_right',
        'front_wheel_joint_left',
        'rear_wheel_joint_right',
        'rear_wheel_joint_left',
    )
    SERVO_JOINT_NAMES = (
        'front_wheel_joint_R',
        'front_wheel_joint_L',
        'rear_wheel_joint_R',
        'rear_wheel_joint_L',
    )

    # Simulated joint -> (hardware name, sign). Sign converts the Gazebo joint
    # convention back to the hardware one (the inverse of the command mapping).
    DRIVE_STATE_MAP = {
        'front_wheel_joint_left': ('drive_left_front', 1.0),
        'middle_wheel_joint_left': ('drive_left_middle', 1.0),
        'rear_wheel_joint_left': ('drive_left_back', 1.0),
        'front_wheel_joint_right': ('drive_right_front', -1.0),
        'middle_wheel_joint_right': ('drive_right_middle', -1.0),
        'rear_wheel_joint_right': ('drive_right_back', -1.0),
    }
    CORNER_STATE_MAP = {
        'front_wheel_joint_L': ('corner_left_front', -1.0),
        'rear_wheel_joint_L': ('corner_left_back', -1.0),
        'front_wheel_joint_R': ('corner_right_front', -1.0),
        'rear_wheel_joint_R': ('corner_right_back', -1.0),
    }

    def __init__(self):
        super().__init__('gazebo_command_adapter')
        self.declare_parameter('trajectory_time_from_start', 0.2)
        # Max wheel acceleration [rad/s^2]. Default ~ real rover: RoboClaw duty
        # accel factor 0.8 -> 0 to full speed (223 rpm = 23.4 rad/s) in ~1.25 s.
        # 0 disables the ramp (commands pass straight through).
        self.declare_parameter('wheel_acceleration', 18.0)
        self.declare_parameter('ramp_rate', 50.0)  # Hz

        n = len(self.WHEEL_JOINT_ORDER)
        self.wheel_target = [0.0] * n
        self.wheel_current = [0.0] * n

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 1)
        self.servo_pub = self.create_publisher(
            JointTrajectory, '/servo_controller/joint_trajectory', 1)

        self.create_subscription(CommandDrive, '/cmd_drive', self.drive_cb, 1)
        self.create_subscription(CommandCorner, '/cmd_corner', self.corner_cb, 1)

        # Simulated encoders: Gazebo joint states -> hardware-style state topics
        self.drive_state_pub = self.create_publisher(JointState, '/drive_state', 1)
        self.corner_state_pub = self.create_publisher(JointState, '/corner_state', 1)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)

        rate = self.get_parameter('ramp_rate').get_parameter_value().double_value
        self.ramp_dt = 1.0 / rate
        self.create_timer(self.ramp_dt, self.ramp_cb)
        self.get_logger().info(
            'Bridging /cmd_drive and /cmd_corner to Gazebo controllers')

    def drive_cb(self, msg: CommandDrive):
        # Undo hardware right-side sign convention for Gazebo joint axes
        # Only store the target here; ramp_cb moves the wheels toward it
        self.wheel_target = [
            -msg.right_middle_vel,
            msg.left_middle_vel,
            -msg.right_front_vel,
            msg.left_front_vel,
            -msg.right_back_vel,
            msg.left_back_vel,
        ]

    def joint_states_cb(self, msg: JointState):
        """Republish simulated joint states the way the real rover's nodes do."""
        self.drive_state_pub.publish(self._remap(msg, self.DRIVE_STATE_MAP))
        self.corner_state_pub.publish(self._remap(msg, self.CORNER_STATE_MAP))

    @staticmethod
    def _remap(msg: JointState, mapping) -> JointState:
        out = JointState()
        out.header = msg.header
        for i, joint in enumerate(msg.name):
            if joint not in mapping:
                continue
            name, sign = mapping[joint]
            out.name.append(name)
            out.position.append(sign * msg.position[i] if i < len(msg.position) else 0.0)
            out.velocity.append(sign * msg.velocity[i] if i < len(msg.velocity) else 0.0)
        return out

    def ramp_cb(self):
        """Step each wheel speed toward its target by at most accel * dt."""
        accel = self.get_parameter('wheel_acceleration').get_parameter_value().double_value
        if accel <= 0.0:
            self.wheel_current = list(self.wheel_target)
        else:
            max_step = accel * self.ramp_dt
            self.wheel_current = [
                cur + max(-max_step, min(max_step, tgt - cur))
                for cur, tgt in zip(self.wheel_current, self.wheel_target)
            ]
        out = Float64MultiArray()
        out.data = list(self.wheel_current)
        self.wheel_pub.publish(out)

    def corner_cb(self, msg: CommandCorner):
        traj = JointTrajectory()
        traj.joint_names = list(self.SERVO_JOINT_NAMES)
        point = JointTrajectoryPoint()
        # Invert motor-frame (z-down) angles into Gazebo joint frames
        point.positions = [
            -msg.right_front_pos,
            -msg.left_front_pos,
            -msg.right_back_pos,
            -msg.left_back_pos,
        ]
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        dt = self.get_parameter('trajectory_time_from_start').get_parameter_value().double_value
        point.time_from_start.sec = int(dt)
        point.time_from_start.nanosec = int((dt % 1.0) * 1e9)
        traj.points = [point]
        self.servo_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCommandAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
