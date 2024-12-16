import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node  # noqa: E402
from launch import LaunchDescription, LaunchService  # noqa: E402

def generate_launch_description():
    compiled = os.environ['need_compile']
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
    camera_nodes = Node(
            package='ascamera', 
            namespace= "ascamera_hp60c",
            executable='ascamera_node', 
            output='both',
            name='ascamera',
           parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "/home/ubuntu/third_party_ros2/third_party_ws/src/ascamera/configurationfiles"},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 15}
        ]
        # remappings=[
              # ('/ascamera_hp60c/ascamera/rgb0/image', '/depth_cam/rgb/image_raw'),
              # ('/ascamera_hp60c/ascamera/depth0/image_raw', '/depth_cam/depth/image_raw'),
             # ]
        )

    return LaunchDescription([camera_nodes])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

