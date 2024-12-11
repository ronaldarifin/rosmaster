from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to sensor launch file
    sensor_launch_path = os.path.join(
        '/root/spongebob/src/sensor_interface/launch', 'sensor_launch.py')

    # Include Sensor Launch
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_path)
    )

    # Add Mcnamu Driver Node
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',  # Replace with your package name if different
        executable='Mcnamu_driver',  # Use the script file directly or the registered executable name    
        )

    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')
    

    # Base Node
    base_node = Node(
        package='yahboomcar_base_node',  # Replace with the appropriate package name
        executable='base_node',          # Replace with the appropriate executable name
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        output='screen'
    )

    imu_filter_config = os.path.join(
         get_package_share_directory('yahboomcar_bringup'), 
         'params',
         'imu_filter_param.yaml'
      )
    
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        name='imu_filter_madgwick'
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_localization'), 'launch'),
         '/ekf.launch.py'])
      )
 
    yahboom_joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('yahboomcar_ctrl'), 'launch'),
         '/yahboomcar_joy_launch.py'])
    )
    
    yahboom_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('yahboomcar_description'), 'launch'),
         '/description_launch.py'])
    )


    # # Add Navigation Node (example placeholder for navigation stack)
    # navigation_node = Node(
    #     package='your_navigation_package',  # Replace with your navigation package
    #     executable='navigation_node',  # Replace with your navigation executable
    #     name='navigation_node',
    #     output='screen',
    #     parameters=[
    #         {'param_name': 'param_value'},  # Replace with actual parameters
    #     ]
    # )

    # Combine everything into a single launch description
    return LaunchDescription([
        sensor_launch,
        mcnamu_driver_node,
        pub_odom_tf_arg,
        base_node,
        imu_filter_node,
        ekf_node,
        yahboom_description_node,
        yahboom_joy_node
        # odometry_node,
        # navigation_node,
    ])
