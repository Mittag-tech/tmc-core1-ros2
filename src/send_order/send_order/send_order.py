#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import serial
import yaml
from pathlib import Path

BASE = str(Path(__file__).parent.parent.parent.parent.parent.parent.parent)


def load_config(config_file=f"{BASE}/src/config.yaml"):
    with open(config_file, mode="r") as file:
        config = yaml.safe_load(file)
    return config["cybergear"], config["servo"]

def calc_ratio(command: int, offset: int, min: int = -128, max: int = 128):
    if command >= offset:
        return (command - offset) / (max - offset)
    elif command < offset:
        return (command - offset) / abs(min - offset)

def calc_cyber(command, mechanum, offset, max_speed):
    # serial dataの上から4つがjoy stickの右左の順番と思って記載
    target_x = max_speed * calc_ratio(command=command[0], offset=offset[0])
    target_y = max_speed * calc_ratio(command=command[1], offset=offset[1])
    target_w = max_speed * calc_ratio(command=command[2], offset=offset[2])
    tr =   target_x + target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    tl = - target_x + target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    bl = - target_x - target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    br =   target_x - target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    return [tr, tl, bl, br]


class PublisherCore(Node):
    def __init__(self, topic_name, port = '/dev/ttyAMA10', baudrate=115200, delay=0.2):
        super().__init__('publisher_core')
        self.publisher = self.create_publisher(Float32MultiArray, topic_name, 10)
        self.topic_name = topic_name
        self.get_logger().info(f'Publishing serial input to {topic_name} for M5')
        
        self.readSer = serial.Serial(port=port, baudrate=baudrate, timeout=3)
        self.timer = self.create_timer(delay, self.publish_serial)

        self.cybergear, self.servo = load_config()

    def publish_serial(self):
        msg = Float32MultiArray()
        try:
            line = self.readSer.readline()
            line = line.strip().decode("utf-8")
            line = [float.fromhex(f"0x{x}") for x in line.split(",")]
        except Exception as e:
            self.get_logger().error(f'データ読み取りエラー: {e}')

        cybergear_command = line[4:8]
        servo_command = line[8:]
        cybergear_data = calc_cyber(command=cybergear_command, 
                                    mechanum=self.cybergear["mechanum"], 
                                    offset=self.cybergear["offset"], 
                                    max_speed=self.cybergear["speed"])
        msg.data = cybergear_data.append(servo_command)
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