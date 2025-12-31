from setuptools import setup

package_name = 'my_ros2_assignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5', 'qtpy'],
    zip_safe=True,
    maintainer='ROS2 Candidate',
    maintainer_email='maintainer@example.com',
    description='GUI-driven Doosan E0509 simulation assignment for ROS 2 Humble.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_ros2_assignment.my_node:main',
        ],
    },
)
