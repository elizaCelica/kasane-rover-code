import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import BatteryState

from ina260.controller import Controller


class INA260Node(Node):
    def __init__(self):
        super().__init__('ina260_node')
        
        # Initialize parameters
        self.declare_parameter('sensor_address', "0x45")
        self.declare_parameter('publish_rate', 2.0)

        # Get parameter values
        self.sensor_address = self.get_parameter('sensor_address').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize INA260 sensor
        self.get_logger().debug(f"Attempting to connect to INA260 at address {self.sensor_address} (int: {int(self.sensor_address, 0)})")
        self.ina260 = Controller(int(self.sensor_address, 0))

        # Create publisher
        self.battery_state_pub = self.create_publisher(BatteryState, 'battery_state', QoSProfile(depth=10))

        # Create timer for publishing at the specified rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f"Started INA260 node. Publishing at {self.publish_rate} Hz.")

    def parameter_callback(self, parameters):
        for param in parameters:
            if param.name == 'publish_rate':
                self.publish_rate = param.value
                # Update the timer period with the new publish rate
                self.timer.timer_period = 1.0 / self.publish_rate
                self.timer.cancel()
                # Create a new timer with the updated period
                self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
                self.get_logger().debug(f"Updated timer to run at {self.publish_rate} Hz")
        return SetParametersResult(successful=True, reason="OK")

    def read_ina(self, tries=30):
        # This I2C bus drops the first transaction after idle time:
        # throwaway read, instant retry on failure, then the real read.
        for _ in range(tries):
            try:
                self.ina260.voltage()
                voltage = self.ina260.voltage()
                current = self.ina260.current()
            except OSError:
                continue
            # A running rover never reads ~0 V; zeros mean a garbled read.
            if voltage > 1.0:
                return voltage, current
        raise OSError("INA260 read failed after retries")

    def publish_data(self):
        try:
            voltage, current = self.read_ina()
        except OSError as e:
            self.get_logger().warning(f"Skipping battery reading: {e}", throttle_duration_sec=10)
            return
        battery_msg = BatteryState()
        battery_msg.voltage = voltage
        battery_msg.current = -current
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN  # Assuming unknown status
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO  # Assuming LiPo technology

        # Publish BatteryState message
        self.battery_state_pub.publish(battery_msg)

def main(args=None):
    rclpy.init(args=args)
    node = INA260Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
