#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/22
# 自检程序(self-test program)
import time
import smbus
from ainex_sdk import buzzer, button

# 传感器测试
def check_sensor():
    bus = smbus.SMBus(1)
    count_imu = 0
    while True:
        count_imu += 1
        try:
            bus.write_byte_data(0x68, 0x6b, 0)
            if button.get_button_status() == 1:
                time.sleep(15)
                buzzer.on()
                time.sleep(0.1)
                buzzer.off()
                print('everything is ok!')
            break
        except Exception as e:
            print(str(e))
        if count_imu > 100:
            print('imu init timeout')
            break
        time.sleep(0.01)

if __name__ == '__main__':
    check_sensor()
