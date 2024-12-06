from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your URDF file
    urdf_file = os.path.join(
        get_package_share_directory('spongebob_description'),
        'urdf',
        'spongebob.urdf'  # Change to spongebob.xacro if using xacro
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # (Optional) RViz node to visualize the robot
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('spongebob_description'),
            'config',
            'spongebob.rviz'  # Optional RViz config file if you have one
        )]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # Uncomment the next line if you have RViz config
        # rviz_node
    ])
