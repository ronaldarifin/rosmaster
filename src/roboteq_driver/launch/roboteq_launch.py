from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboteq_driver',
            executable='roboteq_node',
            name='roboteq_driver',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud_rate': 115200},
                {'max_power': 1000},
            ]
        ),
    ])
