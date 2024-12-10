import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RobotControlSubscriber(Node):
    def __init__(self):
        super().__init__('robot_control_subscriber')

        # Subscriber to listen for commands
        self.subscription = self.create_subscription(
            String,
            '/robot_command',  # Topic name
            self.command_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Robot Control Subscriber Node has been started.")

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        twist = Twist()
        # Map commands to robot motion
        if command == 'forward':
            twist.linear.x = 0.5  # Forward speed
        elif command == 'backward':
            twist.linear.x = -0.5  # Backward speed
        elif command == 'left':
            twist.angular.z = 0.5  # Rotate left
        elif command == 'right':
            twist.angular.z = -0.5  # Rotate right
        elif command == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().warn(f"Unknown command: {command}")

        # Publish the velocity command
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    robot_control_subscriber = RobotControlSubscriber()

    try:
        rclpy.spin(robot_control_subscriber)
    except KeyboardInterrupt:
        pass

    robot_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
