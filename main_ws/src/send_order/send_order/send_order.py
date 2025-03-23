#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import serial
import yaml
import math
from pathlib import Path

import RPi.GPIO as GPIO

BASE = str(Path(__file__).parent.parent.parent.parent)
SHOOT = "B1"
RESET = "B2"
DIRECTION = "T4"
ROLLER = "T3"

# GPIO setting
GPIO.setmode(GPIO.BCM)
OUTPUT_PIN = 22
GPIO.setup(OUTPUT_PIN, GPIO.OUT)


def load_config(config_file=f"{BASE}/src/config.yaml"):
    with open(config_file, mode="r") as file:
        config = yaml.safe_load(file)
    return config["cybergear"], config["servo"]

def calc_ratio(command: int, offset: int, min: int = -128, max: int = 128):
    if command >= offset:
        return (command - offset) / (max - offset)
    elif command < offset:
        return (command - offset) / abs(min - offset)
    
def set_deadzone(data, deadzone):
    if abs(data) <= deadzone:
        return 0.0
    else:
        return data

def calc_cyber(command, mechanum, offset, max_speed, rotate, joystick, deadzone, direction):
    joy_max = joystick["max"]
    joy_min = joystick["min"]
    target_x = max_speed * calc_ratio(command=command[1], offset=offset[1], min=joy_min, max=joy_max)
    target_y = max_speed * calc_ratio(command=command[0], offset=offset[0], min=joy_min, max=joy_max)
    target_w = -rotate * calc_ratio(command=command[3], offset=offset[2], min=joy_min, max=joy_max)
    if not direction:
        target_x = -target_x
        target_y = -target_y
    tr =   target_x + target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    tl = - target_x + target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    bl = - target_x - target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    br =   target_x - target_y + (mechanum[0] + mechanum[1])/mechanum[2] * target_w
    data = [tr, tl, bl, br]
    cybergear_data = [set_deadzone(x, deadzone=deadzone) for x in data]
    return cybergear_data

def bool_toggle(command, mask):
    shift = int(math.log(mask, 2))
    button = int(bin(int(command) & mask), 2)
    servo_command = int(bin(button >> shift), 2)
    if servo_command == 1:
        return True
    else:
        return False
    
def create_servo_data(command, servo, mask, angle_flag, direction):
    roller = bool_toggle(command=command, mask=mask[ROLLER])
    shoot = bool_toggle(command=command, mask=mask[SHOOT])
    reset = bool_toggle(command=command, mask=mask[RESET])
    if not roller:
        roller_pwm = servo["roller_pwm"]["max"]
        if shoot and angle_flag:
            shoot_angle = servo["angle"]
            angle_flag = False
        else:
            shoot_angle = 0
            angle_flag = True
    else:
        roller_pwm = servo["roller_pwm"]["min"]
        shoot_angle = 0
    if reset:
        servo_reset = 1
    else:
        servo_reset = 0
    if not direction:
        servo_data = [shoot_angle, roller_pwm, 0, 0, servo_reset]
    else:
        servo_data = [0, 0, shoot_angle, roller_pwm, servo_reset]
        
    return [float(x) for x in servo_data], angle_flag


class PublisherCore(Node):
    def __init__(self, topic_name, config, port = '/dev/ttyAMA10', baudrate=115200, delay=0.01):
        super().__init__('publisher_core')
        self.publisher = self.create_publisher(Float32MultiArray, topic_name, 10)
        self.topic_name = topic_name
        self.get_logger().info(f'Publishing serial input to {topic_name} for M5')
        
        self.readSer = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.timer = self.create_timer(delay, self.publish_serial)

        self.cybergear, self.servo = load_config(config_file=config)
        self.angle_flag = True

    def publish_serial(self):
        msg = Float32MultiArray()
        try:
            line = self.readSer.readline()
            line = line.strip().decode("utf-8")
            line = [x for x in line.split(",")][3:]
            line = [float.fromhex(f"0x{x}") for x in line]
            if line:
                GPIO.output(OUTPUT_PIN, GPIO.HIGH)
            else:
                self.get_logger().info("connection failed")
                GPIO.output(OUTPUT_PIN, GPIO.LOW)    
            cybergear_command = line[:4]
            servo_command = line[4]
            direction = bool_toggle(command=servo_command,
                                    mask=self.servo["mask"][DIRECTION])
            cybergear_data = calc_cyber(command=cybergear_command, 
                                        mechanum=self.cybergear["mechanum"], 
                                        offset=self.cybergear["offset"], 
                                        max_speed=self.cybergear["speed"],
                                        rotate=self.cybergear["rotate"],
                                        joystick=self.cybergear["joystick"],
                                        deadzone=self.cybergear["deadzone"],
                                        direction=direction)
            servo_data, self.angle_flag= create_servo_data(command=servo_command,
                                                        servo=self.servo,
                                                        mask=self.servo["mask"],
                                                        angle_flag=self.angle_flag,
                                                        direction=direction)
            msg.data = cybergear_data + servo_data
            self.publisher.publish(msg)
            self.get_logger().info(f'Sent to M5: {msg}')
        except Exception as e:
            self.get_logger().error(f'データ読み取りエラー: {e}')
            GPIO.output(OUTPUT_PIN, GPIO.LOW)


    def close(self):
        self.readSer.close()


def main(args=None):
    rclpy.init(args=args)
    
    # トピック名を引数から受け取る（デフォルトは/micro_ros_arduino_subscribe）
    topic_name = '/micro_ros_arduino_subscriber' if len(sys.argv) < 2 else sys.argv[1]
    config = "src/config.yaml" if len(sys.argv) < 2 else sys.argv[2]
    
    node = PublisherCore(topic_name, config=config)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()