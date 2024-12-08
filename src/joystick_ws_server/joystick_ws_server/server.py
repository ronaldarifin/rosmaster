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
        self.get_logger().info('WebSocket server node initialized!')

    async def process_messages(self, websocket, path):
        """Process incoming messages from WebSocket clients."""
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
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    async def websocket_server(self):
        """Run the WebSocket server."""
        try:
            server = await websockets.serve(self.process_messages, '0.0.0.0', 8080)
            self.get_logger().info('WebSocket server running on ws://0.0.0.0:8080')
            await server.wait_closed()
        except Exception as e:
            self.get_logger().error(f'Failed to start WebSocket server: {e}')
            raise


async def ros_spin_async(node):
    """Spin the ROS 2 node in an asyncio-friendly way."""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.1)  # Keep asyncio event loop active
    except Exception as e:
        node.get_logger().error(f'ROS spin error: {e}')


async def main_async():
    """Main async entry point for the WebSocket server and ROS node."""
    rclpy.init()
    node = WebSocketCmdVelNode()

    try:
        await asyncio.gather(
            node.websocket_server(),  # Run the WebSocket server
            ros_spin_async(node),     # Spin the ROS 2 node
        )
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down due to KeyboardInterrupt...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error in main loop: {e}')
    finally:
        node.get_logger().info('Shutting down WebSocketCmdVelNode...')
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Main entry point for the ROS 2 node."""
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
