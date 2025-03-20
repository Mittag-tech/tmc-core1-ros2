from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    port = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port to connect m5stack."
    )

    return LaunchDescription(
        [
            Node(namespace="usb_camera",
                 package="usb_cam",
                 executable="usb_cam_node_exe",
                 ),
            Node(namespace="camera",
                 package="front_camera",
                 executable="front_camera_node",
                 ),
            Node(namespace="micro-ros",
                 package="micro_ros_agent",
                 executable="micro_ros_agent",
                 output="screen",
                 arguments=["serial", "--dev", LaunchDescription("port")]
                 ),
            Node(namespace="cybergear_data",
                 package="send_order",
                 executable="send_order",
                 output="screen",
                 on_exit=Shutdown()
                 ),
        ]
    )