from setuptools import setup

package_name = 'roboteq_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools', 'pyserial'],
    data_files=[
        # Include the resource files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include the package.xml file
        ('share/' + package_name, ['package.xml']),
        # Include the launch files
        ('share/' + package_name + '/launch', ['launch/roboteq_launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'roboteq_node = roboteq_driver.roboteq_node:main',
            'cmd_vel_pub = roboteq_driver.cmd_vel_publisher:main',
        ],
    },
)
