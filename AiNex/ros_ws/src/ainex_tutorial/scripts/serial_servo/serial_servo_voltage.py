#!/usr/bin/env python3
# encoding: utf-8
import time
from ainex_sdk import hiwonder_servo_controller

servo_control = hiwonder_servo_controller.HiwonderServoController('/dev/ttyAMA0', 115200)

print('get serial servo id23 voltage at 10hz')

while True:
    try:
        vol = servo_control.get_servo_vin(23)
        if vol is not None:
            print('\rvoltage: %sV'%(str(vol/1000.0).ljust(6)), end='', flush=True)
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
