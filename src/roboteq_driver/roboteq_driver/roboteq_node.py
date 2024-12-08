#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
from std_msgs.msg import Bool  # For arm/disarm messages


class RoboteqDriverNode(Node):
    def __init__(self):
        super().__init__('roboteq_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_power', 1000)

        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.max_power = self.get_parameter('max_power').get_parameter_value().integer_value

        # Attempt to open serial connection
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            self.get_logger().info(f"Serial port {serial_port} opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
            self.get_logger().warn("Motor commands will not be sent until serial port is available.")

        # Lock for serial communication
        self.lock = threading.Lock()

        # Initialize motor commands
        self.power_left = 0
        self.power_right = 0

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Subscribe to arm_disarm topic
        self.arm_disarm_subscription = self.create_subscription(
            Bool,
            'arm_disarm',
            self.arm_disarm_callback,
            10
        )

        self.get_logger().info("Roboteq driver node started, listening to 'cmd_vel' and 'arm_disarm'.")

    def cmd_vel_callback(self, msg):
        """
        Callback to handle velocity commands and convert them into motor power values.

        Parameters:
        - msg: Twist message with `linear.x` (forward/backward) and `angular.z` (left/right turn) values.
        """
        max_linear_speed = 0.8  # Maximum linear speed in m/s
        max_angular_speed = 0.8  # Maximum angular speed in rad/s

        # Extract linear and angular inputs
        linear_input = msg.linear.x
        angular_input = msg.angular.z

        # Calculate motor speeds
        left_speed = linear_input - angular_input
        right_speed = linear_input + angular_input

        # Normalize motor speeds to max_power range (-max_power to +max_power)
        self.power_left = int((left_speed / max_linear_speed) * self.max_power)
        self.power_right = int((right_speed / max_linear_speed) * self.max_power)

        # Clamp values to max_power range
        self.power_left = max(-self.max_power, min(self.max_power, self.power_left))
        self.power_right = max(-self.max_power, min(self.max_power, self.power_right))

        # Send motor commands
        self.set_motor_power(self.power_left, self.power_right)

    def arm_disarm_callback(self, msg):
        """
        Callback to handle arm/disarm requests.

        Parameters:
        - msg: Bool message (True to arm, False to disarm).
        """
        if msg.data:
            self.arm_controller()
        else:
            self.disarm_controller()

    def arm_controller(self):
        """Send the correct command to clear E-Stop and arm the controller."""
        self.send_command("!MG")  # Clear emergency stop
        self.send_command("!RWD 0")  # Reset watchdog to arm
        self.get_logger().info("Controller armed.")

    def disarm_controller(self):
        """Send the correct command to disarm the controller."""
        self.send_command("!EX")  # Emergency stop to disarm
        self.get_logger().info("Controller disarmed.")


    def set_motor_power(self, power_left, power_right):
        """Send motor power commands to the Roboteq controller."""
        if self.ser is None:
            self.get_logger().warn("Serial port not available. Cannot send motor commands.")
            return

        try:
            command_left = f'!G 1 {power_left}'
            command_right = f'!G 2 {power_right}'
            self.send_command(command_left)
            self.send_command(command_right)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send motor commands: {e}")

    def send_command(self, command):
        """Send a raw command to the Roboteq controller."""
        if self.ser is None:
            return
        full_command = command + '\r'
        with self.lock:
            try:
                self.ser.write(full_command.encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().error(f"Error sending command '{command}': {e}")

    def destroy_node(self):
        """Gracefully shutdown the node and stop the motors."""
        if self.ser is not None:
            self.set_motor_power(0, 0)  # Stop the motors
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboteqDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
