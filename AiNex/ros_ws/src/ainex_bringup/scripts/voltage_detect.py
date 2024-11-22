#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/08/28
# @author:aiden
"""
低压检测，通过读取舵机的电压来判断，当连续60s电压低于10V，蜂鸣器以1次/s的频率提示(low voltage detection. Judge through reading the voltage of the servo.
When the voltage keeps below 10V within 60s. the buzzer will beep once per second)
关闭此功能，重启生效：sudo systemctl disable voltage_detect.service(close this function, take effect after restart)
"""
import time
import threading
from ainex_sdk import buzzer
from hiwonder_servo_driver import hiwonder_servo_io

warning_vol = 10.0  # 低于此值蜂鸣器响

count_low = 0
count_normal = 0
low_vol = False

def vol_warning():
    while True:
        if low_vol:
            buzzer.on() 
            time.sleep(0.5)
            buzzer.off()
            time.sleep(0.5)
        else:
            time.sleep(1)

if __name__ == '__main__':
    buzzer.off()
    threading.Thread(target=vol_warning, daemon=True).start()
    servo_io = hiwonder_servo_io.HiwonderServoIO('/dev/ttyAMA0', 115200)
    try:
        while True:
            result = servo_io.get_servo_vin(23)  # 获取id23舵机电压
            if result is not None:
                vol = round(result/1000.0, 1)
                if vol < warning_vol:
                    count_normal = 0
                    count_low += 1
                else:
                    count_normal += 1
                    count_low = 0
                if count_low > 11:
                    count_low = 1
                    low_vol = True
                if count_normal > 1:
                    low_vol = False
                #print('current voltage: {}V    warning voltage {}V'.format(vol, warning_vol))
            else:
                pass
                # print(result)
            time.sleep(5)
    except KeyboardInterrupt:
        buzzer.off()
