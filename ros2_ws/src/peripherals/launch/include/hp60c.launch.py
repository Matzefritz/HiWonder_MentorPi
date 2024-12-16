from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    ascamera_node = Node(
        namespace= "ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "/home/ubuntu/third_party_ros2/third_party_ws/src/ascamera/configurationfiles"},
            {"color_pcl": True},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 15},
        ],
        remappings=[              
        ('/ascamera_hp60c/ascamera/rgb0/image', '/depth_cam/rgb/image_raw'),
        ('/ascamera_hp60c/ascamera/depth0/image_raw', '/depth_cam/depth0/image_raw'),]
    )

    # ascamera_node2 = Node(
    #     namespace= "ascamera_hp60c_2",
    #     package='ascamera',
    #     executable='ascamera_node',
    #     respawn=True,
    #     output='both',
    #     parameters=[
    #         {"usb_bus_no": 3},    # set your usb_bus_no
    #         {"usb_path": "2.2"},  # set your usb_path
    #         {"confiPath": "./ascamera/configurationfiles"},
    #         {"color_pcl": False},
    #         {"depth_width": 640},
    #         {"depth_height": 480},
    #         {"rgb_width": 640},
    #         {"rgb_height": 480},
    #         {"fps": 15},
    #     ],
    #     remappings=[]
    # )

    ld.add_action(ascamera_node)
    # ld.add_action(ascamera_node2)

    return ld


