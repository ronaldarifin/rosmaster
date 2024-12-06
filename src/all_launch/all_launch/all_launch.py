from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to roboteq launch file
    roboteq_launch_path = os.path.join(
        get_package_share_directory('roboteq_driver'),
        'launch',
        'roboteq_launch.py'
    )

    # Path to URDF launch file
    urdf_launch_path = os.path.join(
        get_package_share_directory('spongebob_description'), 
        'launch',
        'display.launch.py'  # Replace with the actual filename for your URDF launch
    )

    # Include roboteq_driver launch
    roboteq_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roboteq_launch_path)
    )

    # Include URDF launch
    urdf_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_launch_path)
    )

    # Combine both launch descriptions
    return LaunchDescription([
        roboteq_driver,
        urdf_description
    ])
