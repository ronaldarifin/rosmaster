from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the joystick WebSocket server node with configurable settings.
    """
    return LaunchDescription([
        Node(
            package='joystick_ws_server',  # ROS 2 package name
            executable='server',          # Executable defined in setup.py
            name='joystick_ws_server',    # Name of the node
            output='screen',              # Output type: 'log' or 'screen'
            parameters=[
                {'server_host': '0.0.0.0'},  # WebSocket host (bind to all interfaces)
                {'server_port': 8080},       # WebSocket port
                {'log_level': 'info'},       # Logging level: 'debug', 'info', etc.
            ],
        ),
    ])
