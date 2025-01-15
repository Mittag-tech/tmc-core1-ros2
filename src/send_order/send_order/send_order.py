#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(String, 'key_press', 10)
        
        # キーボードリスナーの設定
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        msg = String()
        
        try:
            # 通常のキーの場合
            msg.data = key.char
        except AttributeError:
            # 特殊キー（Shift、Ctrl等）の場合
            msg.data = str(key).replace('Key.', '')
            
        self.publisher.publish(msg)
        self.get_logger().info(f'Key pressed: {msg.data}')
        
        # ESCキーで終了
        if key == keyboard.Key.esc:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    
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