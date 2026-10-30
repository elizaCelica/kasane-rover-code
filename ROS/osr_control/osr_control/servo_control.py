import rclpy
from rclpy.node import Node
import math
from rclpy.parameter import Parameter
import time

# message imports
from sensor_msgs.msg import JointState
from osr_interfaces.msg import CommandCorner, CommandPWM

RAD_TO_DEG = 180 / math.pi


class ServoWrapper(Node):
    """Interface between the PCA9685 controlling the servos and the higher level rover code"""
    corner_motors = ['corner_right_back', 'corner_right_front', 'corner_left_front', 'corner_left_back']

    def __init__(self):
        super().__init__("servo_wrapper")
        self.log = self.get_logger()
        self.log.info("Initializing corner servo controllers")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('centered_pulse_widths', Parameter.Type.INTEGER_ARRAY)
            ]
        )

        # Ensure only one servo wrapper exists
        node_names = self.get_node_names()
        matching_nodes = [n for n in node_names if n == self.get_name()]
        
        if len(matching_nodes) > 1:
            self.log.error(f"Instance of {self.get_name()} already exists! Shutting down.")
            self.destroy_node()
            return
        
        self.servo_actuation_range = 300
        self.centered_pulse_widths = self.get_parameter('centered_pulse_widths').get_parameter_value().integer_array_value
        assert(len(self.centered_pulse_widths) == 4)
        self.deg_per_sec = 200
        # initial values for position estimate (first element) and goal (second element) for each corner motor in deg
        self.corner_state_goal = [(0, 0)] * 4

        self.servo_cmd_msg = CommandPWM()

        self.enc_pub = self.create_publisher(JointState, "/corner_state", 1)
        self.servo_pub = self.create_publisher(CommandPWM, "/cmd_pwm", 10)  # 4 corner commands go out back to back; depth 1 can drop some

        self.corner_cmd_sub = self.create_subscription(CommandCorner, "/cmd_corner", self.corner_cmd_cb, 1)
        self.enc_pub_timer_period = 0.1  # [s]
        self.servo_direction = -1  # set to 1 if the servos are positive pwm clockwise
        self.enc_pub_timer = self.create_timer(self.enc_pub_timer_period, self.publish_encoder_estimate)

        # Hold every corner at center from startup, and keep re-sending the last angles.
        # Without this the servos get no signal until the first steering command and go
        # limp, letting drive torque twist the corners. The PCA9685 interface skips
        # unchanged angles, so the resend costs nothing on the I2C bus unless a write failed.
        self.last_angles = [float(max(min(w, self.servo_actuation_range), 0)) for w in self.centered_pulse_widths]
        self.refresh_timer = self.create_timer(0.5, self.refresh_servos)

    def send_servo(self, ind, angle):
        self.servo_cmd_msg.channel = ind
        self.servo_cmd_msg.duty_cycle = float(angle)
        self.servo_pub.publish(self.servo_cmd_msg)

    def refresh_servos(self):
        for ind in range(4):
            self.send_servo(ind, self.last_angles[ind])

    def corner_cmd_cb(self, cmd: CommandCorner):
        self.log.debug(f"Received corner command message: {cmd}")

        for ind, corner_name in zip(range(4), self.corner_motors):
            # store goal so we can estimate current angle
            angle = getattr(cmd, corner_name[7:]+"_pos") * RAD_TO_DEG
            # TODO make readable, cleaner
            self.corner_state_goal[ind] = (self.corner_state_goal[ind][0], angle)
            # offset to coordinate frame where x points to the middle of the rover, z down
            # and apply middle of actuation range offset, taking into account if servo is positive ccw or cw
            angle = self.centered_pulse_widths[ind] + self.servo_direction * angle
            self.log.debug(f"motor {corner_name} commanded to {angle}")
            # limit to operating range of servo
            angle = max(min(angle, self.servo_actuation_range), 0)

            # publish PCA9685 command
            self.last_angles[ind] = float(angle)
            self.send_servo(ind, angle)


    def publish_encoder_estimate(self):
        """
        Publish an estimate of where each corner motor currently is.
        
        Estimate is based on the last estimate + velocity * time delta if there
        is a difference between goal and current angle
        and is expressed in the motor frame (z down, x forward)
        """
        enc_msg = JointState()
        enc_msg.header.stamp = self.get_clock().now().to_msg()
        for ind, motor_name in zip(range(4), self.corner_motors):
            curr_angle, goal_angle = self.corner_state_goal[ind]
            self.log.debug(f"motor {motor_name}: curr_angle: {curr_angle}, goal: {goal_angle}", throttle_duration_sec=1)
            goal_differential = goal_angle - curr_angle
            velocity = 0
            # compare differential to step size so we can't overshoot and oscillate
            if abs(goal_differential) > self.deg_per_sec * self.enc_pub_timer_period:
                # assume we're running at the desired frequency
                deg_traveled = self.deg_per_sec * self.enc_pub_timer_period
                self.corner_state_goal[ind]= (curr_angle + math.copysign(deg_traveled, goal_differential), goal_angle)
                velocity = math.copysign(self.deg_per_sec, goal_differential)
                
            elif abs(goal_differential) != 0:
                # assume we're pretty much there
                self.corner_state_goal[ind] = (goal_angle, goal_angle)
            
            enc_msg.name.append(motor_name)
            enc_msg.position.append(self.corner_state_goal[ind][0] / RAD_TO_DEG)
            enc_msg.velocity.append(velocity / RAD_TO_DEG)
            enc_msg.effort.append(0)
        self.enc_pub.publish(enc_msg)

def main(args=None):
    rclpy.init(args=args)

    wrapper = ServoWrapper()

    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
