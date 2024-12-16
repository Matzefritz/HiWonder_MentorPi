import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Topic name for lidar scan data'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_frame',
        description='TF frame ID for the lidar'
    )

    # Path to the launch file and package directory
    peripherals_package_path = get_package_share_directory('peripherals')
    lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ldlidar_LD19.launch.py')

    # Include LD19 launch file
    ld19_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            'topic_name': LaunchConfiguration('scan_topic'),
            'frame_id': LaunchConfiguration('lidar_frame')
        }.items()
    )

    return LaunchDescription([
        scan_topic_arg,
        lidar_frame_arg,
        ld19_launch,
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

