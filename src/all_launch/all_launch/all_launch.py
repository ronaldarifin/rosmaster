from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to Velodyne PointCloud launch file
    velodyne_launch_path = os.path.join(
        get_package_share_directory('velodyne'),
        'launch',
        'velodyne-all-nodes-VLP16-launch.py'
    )
    velodyne_dense_path = os.path.join(
        get_package_share_directory('velodyne_dense_filter'),
        'launch',
        'velodyne_dense_filter_launch.py'
    )

    # Path to Roboteq driver launch file
    roboteq_launch_path = os.path.join(
        get_package_share_directory('roboteq_driver'),
        'launch',
        'roboteq_launch.py'
    )

    # Path to URDF description launch file
    urdf_launch_path = os.path.join(
        get_package_share_directory('spongebob_description'),
        'launch',
        'display.launch.py'
    )

    # WebSocket server node
    joystick_ws_server_node = Node(
        package='joystick_ws_server',
        executable='server',
        name='joystick_ws_server',
        output='screen',
        parameters=[
            {'server_host': '0.0.0.0'},
            {'server_port': 8765},
            {'log_level': 'info'},
        ],
    )

    # Include Velodyne PointCloud launch
    velodyne_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_path)
    )

    velodyne_dense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_dense_path)
    )
    
    # Include Roboteq driver launch
    roboteq_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roboteq_launch_path)
    )
    
    # Include URDF description launch
    urdf_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_launch_path)
    )

    # Combine all launch descriptions
    return LaunchDescription([
        velodyne_driver,
        velodyne_dense,
        roboteq_driver,
        joystick_ws_server_node,
        # Uncomment this to include URDF description
        # urdf_description,
    ])
