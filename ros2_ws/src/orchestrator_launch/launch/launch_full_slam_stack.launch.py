import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=[
            'Including launch file located at: ', os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/driver/controller/launch/controller.launch.py')
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/driver/controller/launch/controller.launch.py')),
        ),

        LogInfo(msg=[
            'Including launch file located at: ', os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/peripherals/launch/lidar.launch.py')
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/peripherals/launch/lidar.launch.py')),
        ),

        LogInfo(msg=[
            'Including launch file located at: ', os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/peripherals/launch/usb_cam.launch.py')
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/peripherals/launch/usb_cam.launch.py')),
        ),

        LogInfo(msg=[
            'Including launch file located at: ', os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/peripherals/launch/joystick_control.launch.py')
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.environ['HOME'], 'workspace/ros2_ws/src/peripherals/launch/joystick_control.launch.py')),
        ),

        LogInfo(msg=[
            'Including launch file located at: ', os.path.join(get_package_share_directory('slam_toolbox'), 'launch/online_async_launch.py')
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch/online_async_launch.py')),
        ),
    ])
