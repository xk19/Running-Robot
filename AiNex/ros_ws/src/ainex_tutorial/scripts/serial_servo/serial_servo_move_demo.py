#!/usr/bin/env python3
# encoding: utf-8
import time
from ainex_sdk import hiwonder_servo_controller

servo_control = hiwonder_servo_controller.HiwonderServoController('/dev/ttyAMA0', 115200)

print('id23 serial servo move between 400 - 600')

while True:
    try:
        servo_id = 5  # 舵机id(0-253)
        position = 400  # 位置(0-1000)
        duration = 500  # 时间(20-30000)
        servo_control.set_servo_position(servo_id, position, duration)
        time.sleep(duration/1000.0 + 0.1)  # 完全转到位需要多100ms, 因为数据传输需要时间
        
        position = 600
        duration = 500
        servo_control.set_servo_position(servo_id, position, duration)
        time.sleep(duration/1000.0 + 0.1)
    except KeyboardInterrupt:
        position = 500
        duration = 500
        servo_control.set_servo_position(servo_id, position, duration)
        break
