from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Locate the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('spongebob_description'),
        'urdf',
        'spongebob.urdf'
    )

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # (Optional) Add RViz or other nodes if needed

    return LaunchDescription([
        robot_state_publisher_node
    ])
