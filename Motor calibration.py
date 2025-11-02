# -*- coding: utf-8 -*-
"""
Created on Sun Nov  2 10:48:09 2025

@author: Asmund
"""

import numpy as np
from gpiozero.pins.native import PiGPIOFactory
from gpiozero import Device, PWMOutputDevice as PWM_pin, DigitalOutputDevice as D_pin
import rotary_encoder
import time
import matplotlib.pyplot as plt

Device.pin_factory = PiGPIOFactory()

#pwm range
start_pwm = 0
end_pwm = 1
pwm_steps = 50

motor_dir_state = False
test_duration = 5
dead_time = 1

motor_pin = 11
motor_dir_pin = 12
encoder_A_pin = 13
encoder_B_pin = 14

wheel_diameter = 60 #mm
steps_per_revolution = 4 #directly on motor
motor_gear_ratio = 50/1 #50 to 1


distance = 0.0
mm_per_step = (np.pi*wheel_diameter)/(steps_per_revolution*motor_gear_ratio)


def encoder_increment():
    global distance
    distance += mm_per_step

def encoder_decrement():
    global distance
    distance -= mm_per_step
    
    
motor = PWM_pin(motor_pin)
motor_dir = D_pin(motor_dir_pin, initial_value=motor_dir_state)
rotary_encoder.connect(
    clk_pin=encoder_A_pin,
    dt_pin=encoder_B_pin,
    on_clockwise_turn=encoder_increment,
    on_counter_clockwise_turn=encoder_decrement)


PWM_values = np.linspace(start_pwm, end_pwm, pwm_steps)
speed_values = np.zeros((2, pwm_steps))


for i in range(0, pwm_steps):
    motor.value = PWM_values[i]
    time.sleep(dead_time)
    
    distance = 0
    start_time = time.time()
    time.sleep(test_duration)
    
    speed_values[0][i] = np.abs(distance)/(time.time()-start_time)
    speed_values[1][i] = PWM_values[i]



print(speed_values)    
plt.plot(speed_values[0], speed_values[1])
plt.show()



















