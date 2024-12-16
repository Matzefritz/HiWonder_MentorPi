from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_robot_controller_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('ros_robot_controller_msgs', 'ros_robot_controller_msgs.*')),
)
