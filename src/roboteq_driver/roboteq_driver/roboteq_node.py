#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading

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
            self.get_logger().warn(f"Failed to open serial port {serial_port}: {e}")
            self.get_logger().warn("Will continue without sending motor commands.")

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

        self.get_logger().info("Roboteq driver node started, listening to cmd_vel.")

    def cmd_vel_callback(self, msg):
        # Convert Twist message to motor commands
        linear = msg.linear.x  # Forward/backward
        angular = msg.angular.z  # Left/right turn

        # Adjust these if needed
        max_linear_speed = 1.0   # m/s
        max_angular_speed = 0.0  # rad/s

        # Normalize speeds
        linear_speed = max(-max_linear_speed, min(max_linear_speed, linear))
        angular_speed = max(-max_angular_speed, min(max_angular_speed, angular))

        # Calculate motor powers (since max_angular_speed=0, this is effectively just linear)
        left_speed = linear_speed - angular_speed
        right_speed = linear_speed + angular_speed

        # Avoid division by zero if max_linear_speed + max_angular_speed = 0
        # For now, since angular_speed is zero, this simplifies:
        denominator = max_linear_speed + max_angular_speed
        if denominator == 0:
            denominator = 1.0  # Prevent division by zero, arbitrary choice

        self.power_left = int(left_speed / denominator * self.max_power)
        self.power_right = int(right_speed / denominator * self.max_power)

        # Limit the motor power values
        self.power_left = max(-self.max_power, min(self.max_power, self.power_left))
        self.power_right = max(-self.max_power, min(self.max_power, self.power_right))

        # Send commands to motors if serial is available
        self.set_motor_power(self.power_left, self.power_right)

    def set_motor_power(self, power_left, power_right):
        if self.ser is None:
            self.get_logger().warn("Serial port not available. Cannot send motor commands.")
            return

        command_left = f'!G 1 {power_left}'
        command_right = f'!G 2 {power_right}'
        self.send_command(command_left)
        self.send_command(command_right)

    def send_command(self, command):
        if self.ser is None:
            # Already handled in set_motor_power, but double-check here
            return
        full_command = command + '\r'
        with self.lock:
            self.ser.write(full_command.encode('ascii'))
            # Optionally read response
            # response = self.ser.readline().decode('ascii').strip()
            # self.get_logger().debug(f"Controller response: {response}")

    def destroy_node(self):
        # Stop motors on shutdown if serial is available
        if self.ser is not None:
            self.set_motor_power(0, 0)
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoboteqDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
