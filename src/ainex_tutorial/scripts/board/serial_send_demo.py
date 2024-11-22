#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import platform
from ainex_sdk.gpio import *

if 'raspi' in platform.release():
    serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # 初始化串口， 波特率为115200
else:
    serialHandle = serial.Serial("/dev/ttyTHS1", 115200)  # 初始化串口， 波特率为115200

rx_pin = adapter_board_rx_con #19
tx_pin = adapter_board_tx_con #26

def port_as_write():
    GPIO.output(tx_pin, 1)  # 拉高TX_CON 即 GPIO27
    GPIO.output(rx_pin, 0)  # 拉低RX_CON 即 GPIO17

def port_as_read():
    GPIO.output(rx_pin, 1)  # 拉高RX_CON 即 GPIO17
    GPIO.output(tx_pin, 0)  # 拉低TX_CON 即 GPIO27

def port_init():
    GPIO.setwarnings(False)
    mode = GPIO.getmode()
    if mode == 1 or mode is None:
        GPIO.setmode(GPIO.BCM)
    GPIO.setup(rx_pin, GPIO.OUT)  # 配置RX_CON 即 GPIO17 为输出
    GPIO.output(rx_pin, 0)
    GPIO.setup(tx_pin, GPIO.OUT)  # 配置TX_CON 即 GPIO27 为输出
    GPIO.output(tx_pin, 1)

port_init()

print('serial send "hello" at 10hz')

while True:
    data = b"hello\r\n"
    serialHandle.flushInput()
    port_as_write()
    serialHandle.write(data)
    time.sleep(0.1)
