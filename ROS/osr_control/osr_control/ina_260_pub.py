import rclpy
import subprocess
import time
from collections import deque
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
        self.setup_battery_protection()

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
        self.check_battery(voltage)
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN  # Assuming unknown status
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO  # Assuming LiPo technology

        # Publish BatteryState message
        self.battery_state_pub.publish(battery_msg)


    def setup_battery_protection(self):
        # 4S LiPo: warn ~3.2 V/cell, shut down ~3.0 V/cell. Readings are averaged
        # and must stay low for a while, so voltage sag while driving doesn't trigger it.
        self.declare_parameter('low_voltage_warn', 12.8)
        self.declare_parameter('low_voltage_shutdown', 12.0)
        self.declare_parameter('low_voltage_hold_sec', 10.0)
        self.declare_parameter('shutdown_dry_run', False)
        self.v_warn = float(self.get_parameter('low_voltage_warn').value)
        self.v_shutdown = float(self.get_parameter('low_voltage_shutdown').value)
        self.v_hold = float(self.get_parameter('low_voltage_hold_sec').value)
        self.shutdown_dry_run = bool(self.get_parameter('shutdown_dry_run').value)
        self.v_history = deque(maxlen=10)
        self.below_shutdown_since = None
        self.shutdown_requested = False
        self.get_logger().info(
            f"Battery protection: warn < {self.v_warn:.1f} V, power off < {self.v_shutdown:.1f} V "
            f"for {self.v_hold:.0f} s" + (" (DRY RUN)" if self.shutdown_dry_run else ""))

    def check_battery(self, voltage):
        self.v_history.append(voltage)
        avg = sum(self.v_history) / len(self.v_history)
        now = time.monotonic()
        if avg < self.v_shutdown:
            if self.below_shutdown_since is None:
                self.below_shutdown_since = now
            held = now - self.below_shutdown_since
            self.get_logger().error(
                f"Battery critical: {avg:.2f} V (below {self.v_shutdown:.1f} V for {held:.0f} s)",
                throttle_duration_sec=5)
            if held >= self.v_hold and not self.shutdown_requested:
                self.shutdown_requested = True
                if self.shutdown_dry_run:
                    self.get_logger().error("DRY RUN: would power off the Pi now to protect the battery")
                else:
                    self.get_logger().fatal("Powering off to protect the battery. Unplug it and recharge.")
                    subprocess.Popen(["sudo", "-n", "/usr/bin/systemctl", "poweroff"])
        else:
            self.below_shutdown_since = None
            if avg < self.v_warn:
                self.get_logger().warning(f"Battery low: {avg:.2f} V. Recharge soon.",
                                          throttle_duration_sec=30)

def main(args=None):
    rclpy.init(args=args)
    node = INA260Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
