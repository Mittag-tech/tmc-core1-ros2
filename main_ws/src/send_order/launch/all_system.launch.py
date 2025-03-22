from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port to connect m5stack."
    )
    topic_name = DeclareLaunchArgument(
        "topic_name",
        default_value='/micro_ros_arduino_subscriber',
        description="micro-ros topic name."
    )
    config = DeclareLaunchArgument(
        "config",
        default_value="src/config.yaml"
    )

    return LaunchDescription(
        [
            port,
            topic_name,
            config,
            Node(namespace="usb_camera",
                 package="usb_cam",
                 executable="usb_cam_node_exe",
                 ),
            Node(namespace="camera",
                 package="front_camera",
                 executable="front_camera_node",
                 ),
            Node(namespace="micro_ros",
                 package="micro_ros_agent",
                 executable="micro_ros_agent",
                 output="screen",
                 arguments=["serial", "--dev", LaunchConfiguration("port")]
                 ),
             Node(namespace="cybergear_data",
                 package="send_order",
                 executable="send_order",
                 output="screen",
                 on_exit=Shutdown(),
                 arguments=[LaunchConfiguration("topic_name"), LaunchConfiguration("config")]
                 ),
        ]
    )