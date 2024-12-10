from setuptools import setup
import os
from glob import glob

package_name = 'trajectory_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmaster',
    maintainer_email='jonathango@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam = trajectory_planner.slam:main',
            'robot_controls = trajectory_planner.robot_controls:main',
        ],
    },
)
