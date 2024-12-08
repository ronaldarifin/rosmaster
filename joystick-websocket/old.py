import asyncio
import json
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class WebSocketCmdVelNode(Node):
    def __init__(self):
        super().__init__('websocket_cmd_vel_node')

        # Publisher for /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for /arm_disarm topic
        self.arm_disarm_publisher = self.create_publisher(Bool, '/arm_disarm', 10)

        self.get_logger().info('WebSocketCmdVelNode has been initialized.')

    async def process_messages(self, websocket, path):
        """
        Handle incoming WebSocket messages.

        :param websocket: The WebSocket connection.
        :param path: The URL path of the WebSocket connection.
        """
        self.get_logger().info(f'Client connected from path: {path}')
        try:
            async for message in websocket:
                self.get_logger().debug(f'Received message: {message}')
                try:
                    data = json.loads(message)

                    # Handle arm/disarm commands
                    if 'arm' in data:
                        arm_msg = Bool()
                        arm_msg.data = bool(data['arm'])
                        self.arm_disarm_publisher.publish(arm_msg)
                        state = "armed" if arm_msg.data else "disarmed"
                        self.get_logger().info(f'Published arm_disarm: {state}')
                        continue  # Skip further processing for this message

                    # Handle cmd_vel commands
                    if 'linear' in data or 'angular' in data:
                        twist = Twist()

                        # Extract linear.x if present
                        if 'linear' in data and 'x' in data['linear']:
                            twist.linear.x = float(data['linear']['x'])
                        else:
                            twist.linear.x = 0.0

                        # Extract angular.z if present
                        if 'angular' in data and 'z' in data['angular']:
                            twist.angular.z = float(data['angular']['z'])
                        else:
                            twist.angular.z = 0.0

                        self.cmd_vel_publisher.publish(twist)
                        self.get_logger().info(f'Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

                except (ValueError, TypeError, json.JSONDecodeError) as e:
                    self.get_logger().error(f'Invalid message format: {e}')
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info('Client disconnected.')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    async def start_websocket_server(self, host='0.0.0.0', port=8080):
        """
        Start the WebSocket server.

        :param host: Host address to listen on.
        :param port: Port number to listen on.
        """
        self.get_logger().info(f'Starting WebSocket server on ws://{host}:{port}')
        
        # Use a lambda function to properly bind the instance method
        async with websockets.serve(lambda ws, path: self.process_messages(ws, path), host, port):
            self.get_logger().info('WebSocket server is running.')
            await asyncio.Future()  # Run forever


async def ros_spin(node):
    """
    Spin the ROS node to process callbacks.

    :param node: The ROS 2 node to spin.
    """
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.1)  # Yield control to the event loop


async def main():
    # Initialize ROS 2
    rclpy.init()

    # Create the WebSocketCmdVelNode
    node = WebSocketCmdVelNode()

    try:
        # Run both the ROS spinning and WebSocket server concurrently
        await asyncio.gather(
            ros_spin(node),
            node.start_websocket_server(host='0.0.0.0', port=8080)
        )
    except asyncio.CancelledError:
        # Handle cancellation (e.g., shutdown)
        pass
    except Exception as e:
        node.get_logger().error(f'Error in main: {e}')
    finally:
        # Clean shutdown
        node.get_logger().info('Shutting down WebSocketCmdVelNode...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except Exception as e:
        print(f"Unhandled exception: {e}")
