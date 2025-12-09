import os
from glob import glob
from setuptools import setup

package_name = 'charuco_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mederic Fourmy',
    author_email='mederic.fourmy@gmail.com',
    description='ROS2 node to detect charuco boards.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'charuco_ros2_node = charuco_ros2.charuco_ros2_node:main'
        ],
    },
)