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

    
def calibrate():
    calibration_data = getCalibrationData(80, 10)
    centre = getAproximateCenter(calibration_data)
    
    print(centre)
    

def getCalibrationData(n = 100, t=5):
    input("Rotate the sensor around collecting data, hit enter to start data collection.")
    
    data = []
    for i in range(n):
        print('Reading ' + str(i+1) + '/' + str(n))
        data.append(readRawAxisData())
        time.sleep(t/n)
    
    return data

def squaredDistance(a, b):
    return (b[0]-a[0])**2 + (b[1]-a[1])**2 + (b[2]-a[2])**2

def score(c, points):
    minD = maxD = squaredDistance(c, points[0])
    
    for p in points:
        d = squaredDistance(c, p)
        minD = min(minD, d)
        maxD = max(maxD, d)
    
    #return math.sqrt(maxD)-math.sqrt(minD)
    return maxD-minD


def getAproximateCenter(points, step = 200):
    #average position is used as a starting point
    c = [0,0,0]
    for p in points:
        c[0] = p[0]
        c[1] = p[1]
        c[2] = p[2]
    c[0] /= len(points)
    c[1] /= len(points)
    c[2] /= len(points)
    
    best_score = score(c, points)
    best_c = c[:]
    
    while True:
        for x in range(-step, step, step):
            for y in range(-step, step, step):
                for z in range(-step, step, step):
                    t = c[:]
                    t[0] += x
                    t[1] += y
                    t[2] += z
                    
                    s = score(t, points)
                    if(s < best_score):
                        best_score = s
                        best_c = t[:]
                        
        
        if best_c == c:
            break
        
        c = best_c[:]
    return c

def readRawAxisData():
    xl = bus.read_byte_data(DEVICE_ADDRESS, OUTX_L_REG_M)
    xh = bus.read_byte_data(DEVICE_ADDRESS, OUTX_H_REG_M)
    yl = bus.read_byte_data(DEVICE_ADDRESS, OUTY_L_REG_M)
    yh = bus.read_byte_data(DEVICE_ADDRESS, OUTY_H_REG_M)
    zl = bus.read_byte_data(DEVICE_ADDRESS, OUTZ_L_REG_M)
    zh = bus.read_byte_data(DEVICE_ADDRESS, OUTZ_H_REG_M)

    x = _twos_comp(((xh & 0xff)<<8) | xl, 16) # + the x offset. See below.
    y = _twos_comp(((yh & 0xff)<<8) | yl, 16) # plus the y offset and everything times the y scale. See below.
    z = _twos_comp(((zh & 0xff)<<8) | zl, 16)

    return (-x*150, -y*150, z*150)

def getHeading():
    values = readRawAxisData()
    return math.degrees(math.atan2(values[1], values[0]))