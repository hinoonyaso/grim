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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Assignment Author',
    maintainer_email='example@example.com',
    description='Doosan E0509 robot arm simulation with ROS2 and PyQt5 GUI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_ros2_assignment.my_node:main',
        ],
    },
)
