from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    port = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port to connect m5stack."
    )
    front_device_arg = DeclareLaunchArgument(
        'front_device', default_value=TextSubstitution(text='/dev/video0'),
        description='Device path for camera 1'
    )
    
    rear_device_arg = DeclareLaunchArgument(
        'rear_device', default_value=TextSubstitution(text='/dev/video2'),
        description='Device path for camera 2'
    )
    
    # カメラネームスペースの設定
    camera_namespaces = ['front', 'rear']

    return LaunchDescription(
        [
          port,
          front_device_arg,
          rear_device_arg,
          
          # 1台目のカメラノード
          Node(
               package='usb_cam',
               executable='usb_cam_node_exe',
               namespace='front',
               parameters=[{
                    'video_device': LaunchConfiguration('front_device'),
                    'frame_id': 'front_frame',
                    'camera_name': 'front',
                    'image_width': 640,
                    'image_height': 480,
                    'pixel_format': 'yuyv'
               }]
          ),
          
          # 2台目のカメラノード
          Node(
               package='usb_cam',
               executable='usb_cam_node_exe',
               namespace='rear',
               parameters=[{
                    'video_device': LaunchConfiguration('rear_device'),
                    'frame_id': 'rear_frame',
                    'camera_name': 'rear',
                    'image_width': 640,
                    'image_height': 480,
                    'pixel_format': 'yuyv'
               }]
          ),
          Node(namespace="camera",
               package="front_camera",
               executable="front_camera_node",
               parameters=[{
                    'camera_ns': camera_namespaces
               }],
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