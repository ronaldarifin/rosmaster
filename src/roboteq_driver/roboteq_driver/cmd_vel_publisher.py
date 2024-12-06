#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("cmd_vel publisher node started.")
        self.linear_speed = 0.5  # Adjust as needed
        self.angular_speed = 0.0  # Adjust as needed

    def timer_callback(self):
        # Example: Send forward motion
        msg = Twist()
        msg.linear.x = self.linear_speed  # Forward speed
        msg.angular.z = self.angular_speed  # Angular speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
