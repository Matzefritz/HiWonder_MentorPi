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

    # 声明节点(declare node)
    a1_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[
            {'channel_type': 'serial',
            'serial_baudrate': 115200,
            'serial_port': '/dev/lidar',  # 统一映射为/dev/lidar(uniformly mapping to /dev/lidar)
            'frame_id': lidar_frame,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'}
        ],
        remappings=[('scan', scan_raw)]
    )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        a1_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

