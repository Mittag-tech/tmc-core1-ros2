from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    web_node = Node(
        package='dual_webcam',
        executable='web_cam_node',
        output='screen'
    )

    return LaunchDescription([web_node])

