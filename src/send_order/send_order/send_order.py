#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import serial
import numpy as np


class PublisherCore(Node):
    def __init__(self, topic_name, port = '/dev/ttyACM0', baudrate=115200, delay=0.01):
        super().__init__('publisher_core')
        self.publisher = self.create_publisher(Float32MultiArray, topic_name, 10)
        self.topic_name = topic_name
        self.get_logger().info(f'Publishing serial input to {topic_name} for M5')
        
        self.cybergear_data = list(np.zeros(4))
        self.servo_data = list(np.zeros(4))

        self.readSer = serial.Serial(port=port, baudrate=baudrate, timeout=3)
        self.timer = self.create_timer(delay, self.publish_serial)

    def publish_serial(self):
        msg = Float32MultiArray()
        try:
            line = self.readSer.readline()
            line = line.strip().decode("utf-8")
            line = [float.fromhex(f"0x{x}") for x in line.split(",")]
        except Exception as e:
            self.get_logger().error(f'データ読み取りエラー: {e}')
            
        msg.data = line
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent to M5: {msg}')

    def close(self):
        self.readSer.close()


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
        node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()