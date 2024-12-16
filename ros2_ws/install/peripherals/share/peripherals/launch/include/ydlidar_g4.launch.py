from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数(declare parameter)
    lidar_frame = LaunchConfiguration('lidar_frame', default='lidar_frame')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)

    # G4节点(G4 node)
    g4_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[
            {'port': '/dev/lidar',
            'frame_id': lidar_frame,
            'ignore_array': '',
            'baudrate': 230400,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 9,
            'intensity_bit': 0,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'auto_reconnect': True,
            'reversion': True,
            'inverted': True,
            'isSingleChannel': False,
            'intensity': False,
            'support_motor_dtr': False,
            'invalid_range_is_inf': False,
            'angle_min': -180.0,
            'angle_max': 180.0,
            'range_min': 0.12,
            'range_max': 16.0,
            'frequency': 12.0}
        ],
        remappings=[('scan', scan_raw)]
    )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        g4_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


