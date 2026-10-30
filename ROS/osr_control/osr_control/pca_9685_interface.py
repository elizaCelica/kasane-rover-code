import rclpy
from rclpy.node import Node
from enum import Enum
from math import ceil

# project libraries
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# message imports
from osr_interfaces.msg import CommandPWM

class ChannelType(Enum):
    SERVO = 0
    CONTINUOUS = 1
    GENERIC = 2

class PCA9685Interface(Node):
    def __init__(self):
        super().__init__("pca9685_interface",
                         automatically_declare_parameters_from_overrides=True)
        self.log = self.get_logger()
        self.pca = None

        # Ensure only one PCA9685 interface exists
        # compare name AND namespace, so the same node can run on several robots
        matching_nodes = [n for n in self.get_node_names_and_namespaces()
                          if n == (self.get_name(), self.get_namespace())]
        
        if len(matching_nodes) > 1:
            self.log.error(f"Instance of {self.get_name()} already exists! Shutting down.")
            self.destroy_node()
            return

        self.channel_params = self.get_parameters_by_prefix('channel_mapping')
        self.params_to_dict()

        self.cmd_pca_sub = self.create_subscription(CommandPWM, "cmd_pwm", self.pca9685_cmd_cb, 10)

        # Keep track of last-commanded signal for each channel to avoid commanding the same duty cycle over and over
        self.last_signal = [None] * 16

        # Account for 16 possible servos
        self.servos = [None] * 16

        # This rover's I2C bus drops the first transaction after any idle period,
        # so setup is retried immediately instead of failing the node.
        for attempt in range(30):
            try:
                self.connect_pca9685()
                self.channel_setup()
                break
            except (OSError, ValueError) as e:
                self.pca = None
                self.log.warning(f"PCA9685 init attempt {attempt + 1} failed: {e}")
        if not self.pca:
            raise RuntimeError("Could not initialize PCA9685 after retries")

        # Periodically rewrite the last servo positions so a write lost on the bus,
        # or a PCA9685 reset, can't leave a servo without a signal.
        self.refresh_timer = self.create_timer(0.5, self.refresh_servos)

    def __del__(self):
        if self.pca:
            self.pca.deinit()

    def write_servo(self, channel, value, tries=30):
        """Write a servo position, retrying immediately if the I2C write fails."""
        target_servo = self.servos[channel]
        for _ in range(tries):
            try:
                if isinstance(target_servo, servo.ContinuousServo):
                    target_servo.throttle = float(value)
                else:
                    target_servo.angle = float(value)
                return True
            except OSError:
                continue
        self.log.warning(f"Failed to set channel {channel} after {tries} tries", throttle_duration_sec=5)
        return False

    def refresh_servos(self):
        for channel, value in enumerate(self.last_signal):
            if value is not None and self.servos[channel] is not None:
                self.write_servo(channel, value)

    def connect_pca9685(self):

        self.log.debug("Creating PCA9685 control instance")
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

    def params_to_dict(self):

        map_dict = {}
        for param_key, param_obj in self.channel_params.items():
            item_id, attr = param_key.split('.', 1)
            if item_id not in map_dict:
                map_dict[item_id] = {}
            map_dict[item_id][attr] = param_obj.value

        self.channel_params = map_dict

    def channel_setup(self):

        for params in self.channel_params.values():

            channel = params['channel']
            self.log.info(f"Configuring hardware for channel {channel}")

            ch_type = ChannelType(params['type'])
            if ch_type is ChannelType.GENERIC:
                continue

            range = params['actuation_range']
            pulse_widths = params['pulse_width_range']

            if(ch_type is ChannelType.SERVO):
                self.servos[channel] = servo.Servo(self.pca.channels[channel],
                                                   actuation_range=range,
                                                   min_pulse=pulse_widths[0],
                                                   max_pulse=pulse_widths[1])
            elif(ch_type is ChannelType.CONTINUOUS):
                self.servos[channel] = servo.ContinuousServo(self.pca.channels[channel],
                                                   actuation_range=range,
                                                   min_pulse=pulse_widths[0],
                                                   max_pulse=pulse_widths[1])

    def pca9685_cmd_cb(self, cmd: CommandPWM):

        if not self.pca:
            return

        channel = cmd.channel

        if channel < 0 or channel > 15:
            self.log.warn(f"Channel {channel} is out of range, dropping command", throttle_duration_sec=1)
            return

        last_signal = self.last_signal[channel]

        if cmd.duty_cycle != last_signal:

            # handle standard and continuous servos separately from generic PWM
            if self.servos[channel] is not None:
                try:
                    if self.write_servo(channel, cmd.duty_cycle):
                        self.last_signal[channel] = cmd.duty_cycle
                except ValueError:
                    # Handle out of range without killing the node
                    self.log.warn(f"Angle {cmd.duty_cycle} out of range for channel {channel}", throttle_duration_sec=1)
                except Exception as e:
                    self.log.error(f"Servo Error: {e}")
                finally:
                    return

            try:
                duty_cycle = ceil((cmd.duty_cycle / 100) * 65535)

                self.pca.channels[channel].duty_cycle = duty_cycle
                self.last_signal[channel] = cmd.duty_cycle

            except Exception as e:
                self.log.error(f"PWM Error: {e}")

def main(args=None):

    rclpy.init(args=args)

    wrapper = PCA9685Interface()

    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()