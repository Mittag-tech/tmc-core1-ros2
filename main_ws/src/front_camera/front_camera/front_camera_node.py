import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError


class ImgProcOpenCVROS(Node):

    def __init__(self):
        super().__init__('imgproc_opencv_ros')
        self.declare_parameter('camera_ns', ['front', 'rear'])
        camera_namespaces = self.get_parameter('camera_ns').value
        
        self.get_logger().info(f'Initializing front camera node with {len(camera_namespaces)} cameras')
        
        # cv_bridgeのインスタンス化
        self.cv_bridge = CvBridge()
        
        # 各カメラのサブスクライバーとパブリッシャーを設定
        self.subscribers = []
        self.publishers = {}
        
        for camera_ns in camera_namespaces:
            self.get_logger().info(f'Setting up camera: {camera_ns}')
            
            # サブスクライバーの作成
            sub = self.create_subscription(
                Image,
                f'/{camera_ns}/image_raw',
                lambda msg, cam=camera_ns: self.image_callback(msg, cam),
                10
            )
            self.subscribers.append(sub)
            
            # パブリッシャーの作成
            pub = self.create_publisher(
                Image,
                f'/processed/{camera_ns}/image',
                10
            )
            self.publishers[camera_ns] = pub

    def image_callback(self, msg, camera_ns):
        self.get_logger().debug(f'Received image from {camera_ns}')
        
        try:
            # ROSイメージメッセージをOpenCVイメージに変換
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, f'Camera: {camera_ns}', (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(cv_image, f'Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}', 
                       (10, 70), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            
            # 処理した画像をROS形式に戻してパブリッシュ
            processed_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            processed_msg.header = msg.header  # 元のヘッダー情報を保持
            self.publishers[camera_ns].publish(processed_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')


def main():
    rclpy.init()
    imgproc_opencv_ros = ImgProcOpenCVROS()
    try:
        rclpy.spin(imgproc_opencv_ros)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()