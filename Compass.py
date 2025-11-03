# -*- coding: utf-8 -*-
"""
Created on Mon Nov  3 16:22:50 2025

@author: guy in forum: https://forums.raspberrypi.com/viewtopic.php?t=123737
"""

import smbus
import time
import math

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x1e
REGISTER_CRA_REG_M = 0x00
REGISTER_MR_REG_M = 0x02
REGISTER_OUT_X_H_M = 0x03
REGISTER_OUT_X_LH_M = 0x04
REGISTER_OUT_Z_H_M = 0x05
REGISTER_OUT_Z_L_M = 0x06
REGISTER_OUT_Y_L_M = 0X08
REGISTER_OUT_Y_H_M = 0X07

def getX():
    xl = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_OUT_X_LH_M)
    xh = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_OUT_X_H_M)
    x =  (xh << 8) | xl
    if x >= 32768:
        x = x ^ 65535
        x += 1
        return -x
    return x

def getZ():
    zl = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_OUT_Z_L_M)
    zh = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_OUT_Z_H_M)
    z = (zh << 8) | zl
    if z >= 32768:
        z = z ^ 65535
        z += 1
        return -z
    return z

def getY():
    yl = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_OUT_Y_L_M)
    yh = bus.read_byte_data(DEVICE_ADDRESS, REGISTER_OUT_Y_H_M)
    y = (yh << 8) | yl
    if y >= 32768:
        y = y ^ 65535
        y += 1
        return -y
    return y

bus.write_byte_data(DEVICE_ADDRESS, REGISTER_CRA_REG_M, 0x90)
bus.write_byte_data(DEVICE_ADDRESS, REGISTER_MR_REG_M, 0)

try:
    while True:
        x = getX()
        y = getY()
        z = getZ()
        print(math.degrees(math.atan2(y, x)))
        time.sleep(1)
except KeyboardInterrupt:
    pass