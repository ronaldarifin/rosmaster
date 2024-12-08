from setuptools import setup

package_name = 'joystick_ws_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This ensures the 'joystick_ws_server' directory is included
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_server_launch.py']),
    ],
    install_requires=['setuptools', 'websockets'],  # Include 'websockets' if used in the server
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='WebSocket server for joystick data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = joystick_ws_server.server:main',  # Ensure the entry point is correct
        ],
    },
)
