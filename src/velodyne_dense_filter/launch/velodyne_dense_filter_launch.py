from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_dense_filter',
            executable='filter_dense_node',
            name='velodyne_dense_filter_node',
            output='screen',
            parameters=[
                {'input_topic': '/velodyne_points'},
                {'output_topic': '/velodyne_dense_points'}
            ]
        ),
    ])
