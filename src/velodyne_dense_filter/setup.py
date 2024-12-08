from setuptools import find_packages, setup

package_name = 'velodyne_dense_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/velodyne_dense_filter_launch.py']),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teng',
    maintainer_email='jonathango@berkeley.edu',
    description='Filter Velodyne point clouds to ensure dense format',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter_dense_node = velodyne_dense_filter.filter_dense_node:main',
        ],
    },
)
