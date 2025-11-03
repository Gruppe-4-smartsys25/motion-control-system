# -*- coding: utf-8 -*-
"""
Created on Mon Nov  3 16:22:50 2025

@author: guy in forum: https://forums.raspberrypi.com/viewtopic.php?t=123737
(and Åsmund a little)
"""

#startup sequence, mag
#1. Write CFG_REG_A_M = 00h // Mag = 10 Hz (high-resolution and continuous mode)
#2. Write CFG_REG_C_M = 01h // Mag data-ready interrupt enable

#extra
# CFG_REG_B_M = 03h, enable low pass filter and offset cancelation

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

x_offset = 0
x_scale = 1
y_offset = 0
y_scale = 1
z_offset = 0
z_scale = 1

def _twos_comp(val, bits):
    if (val & (1<<(bits-1))) != 0:
        return val - (1<<bits)
    return val

def startup(x_off=0, x_amp=1, y_off=0, y_amp=1, z_off=0, z_amp=1):
    global x_offset
    global x_scale
    global y_offset
    global y_scale
    global z_offset
    global z_scale
    x_offset = x_off
    x_scale = x_amp
    y_offset = y_off
    y_scale = y_amp
    z_offset = z_off
    z_scale = z_amp
    
    bus.write_byte_data(DEVICE_ADDRESS, CFG_REG_A_M, 0x00)
    bus.write_byte_data(DEVICE_ADDRESS, CFG_REG_B_M, 0x03)
    bus.write_byte_data(DEVICE_ADDRESS, CFG_REG_C_M, 0x01)

def getOffsets(readings = 100, desired_amp = 1):
    
    vals = [0]*6
    order = ['x', 'y', 'z']
    direction = ['north', 'south']
    for i in range(6):
        input("turn " + str(order[int(i/2)]) + " towards " + direction[i%2] + ", then press enter")
        for i in range(readings):
            vals[i] += readAxisData()[0]
            time.sleep(0.005)
        vals[i] /= readings
    
    x_o = -(vals[0]+vals[1])/2
    x_s = desired_amp/(vals[0] + x_o)
    y_o = -(vals[2]+vals[3])/2
    y_s = desired_amp/(vals[2] + y_o)
    z_o = -(vals[4]+vals[5])/2
    z_s = desired_amp/(vals[4] + z_o)
    
    return (x_o, x_s, y_o, y_s, z_o, z_s)
    
    
    

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