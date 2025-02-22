#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pynput import keyboard
import sys

class PublisherCore(Node):
    def __init__(self, topic_name):
        super().__init__('publisher_core')
        self.publisher = self.create_publisher(Float32MultiArray, topic_name, 10)
        
        # キーボードリスナーの設定
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        
        self.topic_name = topic_name
        self.get_logger().info(f'Publishing keyboard input to {topic_name} for M5')

    def on_press(self, key):
        msg = Float32MultiArray()
        
        try:
            # 通常のキーの場合
            key_char = key.char
            msg.data = [0.0, 0.0]
        except AttributeError:
            # 特殊キー（Shift、Ctrl等）の場合
            special_keys = {
                'Key.up': 1,
                'Key.down': 2,
                'Key.left': 3,
                'Key.right': 4,
                'Key.space': 32,
                'Key.enter': 13
            }
            msg.data = [float(special_keys.get(str(key), 0)), 1.0]

        self.publisher.publish(msg)
        self.get_logger().info(f'Sent to M5: {msg}')
        
        # ESCキーで終了
        if key == keyboard.Key.esc:
            return False

def main(args=None):
    rclpy.init(args=args)
    
    # トピック名を引数から受け取る（デフォルトは/micro_ros_arduino_subscribe）
    topic_name = '/micro_ros_arduino_subscriber' if len(sys.argv) < 2 else sys.argv[1]
    
    node = PublisherCore(topic_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()