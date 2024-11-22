#!/usr/bin/env python3
# encoding: utf-8
import time
from ainex_sdk import hiwonder_servo_controller

servo_control = hiwonder_servo_controller.HiwonderServoController('/dev/ttyAMA0', 115200)

print('get serial servo status')

while True:
    try:
        servo_id = 23
        #servo_id = servo_control.get_servo_id()  # 如果只连接一个舵机且id未知
        pos = servo_control.get_servo_position(servo_id)
        dev = servo_control.get_servo_deviation(servo_id)
        if dev is not None:
            if dev > 125:
                dev = -(0xff - (dev - 1))
        else:
            dev = 0
            print('get dev error')
        angle_range = servo_control.get_servo_range(servo_id)
        vin_range = servo_control.get_servo_vin_range(servo_id)
        temperature_warn = servo_control.get_servo_temp_range(servo_id)
        temperature = servo_control.get_servo_temp(servo_id)
        vin = servo_control.get_servo_vin(servo_id)
        load_state = servo_control.get_servo_load_state(servo_id)

        print('id:%s'%(str(servo_id).ljust(3)))
        print('pos:%s'%(str(pos).ljust(4)))
        print('dev:%s'%(str(dev).ljust(4)))
        print('angle_range:%s'%(str(angle_range).ljust(4)))
        print('voltage_range:%smV'%(str(vin_range).ljust(5)))
        print('temperature_warn:%s°C'%(str(temperature_warn)))
        print('temperature:%s°C'%(str(temperature)))
        print('vin:%smV'%(str(vin).ljust(4)))
        print('lock:%s'%(str(load_state).ljust(4)))
        print('------------------------------')
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
