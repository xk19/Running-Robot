#!/usr/bin/python3
# coding=utf8
from ainex_sdk.gpio import *

GPIO.setup(adapter_board_key, GPIO.IN)  # 设置引脚为输入模式(set the pin as input)

key_dict = {"key": adapter_board_key}

def get_button_status():
    return GPIO.input(adapter_board_key)

if __name__ == '__main__':
    while True:
        print(get_button_status())
