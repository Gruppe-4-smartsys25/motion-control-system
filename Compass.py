# -*- coding: utf-8 -*-
"""
Created on Mon Nov  3 16:22:50 2025

@author: guy in forum: https://forums.raspberrypi.com/viewtopic.php?t=123737
(and Åsmund a little)
"""

"""
From microbit lsm303 driver: https://github.com/lancaster-university/microbit-dal/blob/master/source/drivers/LSM303Magnetometer.cpp
And LSM303AGR datasheet: https://www.st.com/resource/en/datasheet/lsm303agr.pdf

config
CFG_REG_A_M = 0x00/0x04/0x08/0x0C  -only setting ODR
CFG_REG_C_M = 0x01 -configures the INT_MAG/DRDY as a digital output, not something we strictly need

when reading
Register: OUTX_L_REG_M | 0x80, the 0x80 sets the MSB high, this enables auto increment
    (LSM303AGR datasheet, 6.1.1 I2C operation, 3rd paragraph)
    

Swaps and inverts x and y, probably to align more sensibly with the microbit
Also "normalizes" each axis, by multiplying them by 150


Need to look more into the calibration done
"""

import smbus
import math
import time

DEVICE_ADDRESS = 0x1e

CFG_REG_A_M = 0x60
CFG_REG_B_M = 0x61
CFG_REG_C_M = 0x62
OUTX_L_REG_M = 0x68
OUTX_H_REG_M = 0x69
OUTY_L_REG_M = 0x6A
OUTY_H_REG_M = 0x6B
OUTZ_L_REG_M = 0x6C
OUTZ_H_REG_M = 0x6D

bus = smbus.SMBus(1)


def _twos_comp(val, bits):
    if (val & (1<<(bits-1))) != 0:
        return val - (1<<bits)
    return val

def startup():
    bus.write_byte_data(DEVICE_ADDRESS, CFG_REG_A_M, 0x00)
    bus.write_byte_data(DEVICE_ADDRESS, CFG_REG_B_M, 0x03)
    bus.write_byte_data(DEVICE_ADDRESS, CFG_REG_C_M, 0x01)

    
    
    

def readAxisData():
    xl = bus.read_byte_data(DEVICE_ADDRESS, OUTX_L_REG_M)
    xh = bus.read_byte_data(DEVICE_ADDRESS, OUTX_H_REG_M)
    yl = bus.read_byte_data(DEVICE_ADDRESS, OUTY_L_REG_M)
    yh = bus.read_byte_data(DEVICE_ADDRESS, OUTY_H_REG_M)
    zl = bus.read_byte_data(DEVICE_ADDRESS, OUTZ_L_REG_M)
    zh = bus.read_byte_data(DEVICE_ADDRESS, OUTZ_H_REG_M)

    x = _twos_comp(((xh & 0xff)<<8) | xl, 16) # + the x offset. See below.
    y = _twos_comp(((yh & 0xff)<<8) | yl, 16) # plus the y offset and everything times the y scale. See below.
    z = _twos_comp(((zh & 0xff)<<8) | zl, 16)

    return (x, y, z)

def getHeading():
    values = readAxisData()
    return math.degrees(math.atan2(values[1], values[0]))