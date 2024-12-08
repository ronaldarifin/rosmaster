import asyncio
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import json


class WebSocketCmdVelNode(Node):
    def __init__(self):
        super().__init__('websocket_cmd_vel_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_disarm_publisher = self.create_publisher(Bool, '/arm_disarm', 10)
        self.get_logger().info('WebSocket server initialized!')

    async def process_messages(self, websocket):
        self.get_logger().info('WebSocket client connected')
        try:
            async for message in websocket:
                try:
                    # Parse the received WebSocket message
                    data = json.loads(message)

                    # Check for arm/disarm commands
                    if 'arm' in data:
                        arm_message = Bool()
                        arm_message.data = bool(data['arm'])  # True for arm, False for disarm
                        self.arm_disarm_publisher.publish(arm_message)
                        state = "armed" if arm_message.data else "disarmed"
                        self.get_logger().info(f'Controller {state}')
                        continue  # Skip further processing for this message

                    # Handle cmd_vel messages
                    if 'linear' in data or 'angular' in data:
                        twist = Twist()
                        twist.linear.x = float(data.get('linear', {}).get('x', 0.0))
                        twist.angular.z = float(data.get('angular', {}).get('z', 0.0))
                        self.cmd_vel_publisher.publish(twist)
                        self.get_logger().info(f'Published: linear.x={twist.linear.x}, angular.z={twist.angular.z}')
                except (ValueError, TypeError, json.JSONDecodeError) as e:
                    self.get_logger().error(f'Failed to process message: {e}')
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info('WebSocket client disconnected')

    async def websocket_server(self):
        server = await websockets.serve(self.process_messages, '0.0.0.0', 8080)
        self.get_logger().info('WebSocket server running on ws://0.0.0.0:8080')
        await server.wait_closed()


async def main_async():
    # Initialize ROS 2 node
    rclpy.init()
    node = WebSocketCmdVelNode()

    # Run WebSocket server and ROS 2 node together
    try:
        await asyncio.gather(
            node.websocket_server(),
            ros_spin_async(node),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


async def ros_spin_async(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.1)  # Keep asyncio event loop active


if __name__ == '__main__':
    asyncio.run(main_async())
