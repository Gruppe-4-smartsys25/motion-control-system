# -*- coding: utf-8 -*-
"""
Created on Mon Sep 15 00:21:36 2025

@author: Åsmund
"""

import numpy as np
import rotary_encoder
from gpiozero.pins.native import PiGPIOFactory
from gpiozero import Device, PWMOutputDevice as PWM_pin, DigitalOutputDevice as D_pin
import adafruit_lsm303dlh_mag
import board
import time

class Aroweek:
    wheel_diameter = 60 #mm
    steps_per_revolution = 4 #directly on motor
    motor_gear_ratio = 50/1 #50 to 1
    
    wheel_centre_to_centre = [250, 300]
    
    max_velocity = 100 #mm/s
    acceleration = 50 #mm/s
    
    target_angle_tollerance = 0.01*2*np.pi #radian
    
    mm_per_step = (np.pi*wheel_diameter)/(steps_per_revolution*motor_gear_ratio)
    stationary_turn_radius = 0.5*np.sqrt(wheel_centre_to_centre[0]**2 + wheel_centre_to_centre[1]**2)
    tangential_turning_velocity =  wheel_centre_to_centre/(2*stationary_turn_radius)
    
    speed_curve = [[0.00000000e+00,5.26315789e+00,1.05263158e+01,1.57894737e+01,
      2.10526316e+01,2.63157895e+01,3.15789474e+01,3.68421053e+01,
      4.21052632e+01,4.73684211e+01,5.26315789e+01,5.78947368e+01,
      6.31578947e+01,6.84210526e+01,7.36842105e+01,7.89473684e+01,
      8.42105263e+01,8.94736842e+01,9.47368421e+01,1.00000000e+02],
     [0.00000000e+00,5.26315789e-02,1.05263158e-01,1.57894737e-01,
      2.10526316e-01,2.63157895e-01,3.15789474e-01,3.68421053e-01,
      4.21052632e-01,4.73684211e-01,5.26315789e-01,5.78947368e-01,
      6.31578947e-01,6.84210526e-01,7.36842105e-01,7.89473684e-01,
      8.42105263e-01,8.94736842e-01,9.47368421e-01,1.00000000e+00]]
    

    def __init__(self, left_wheel_pin, left_dir_pin, left_encoder_pin_A, left_encoder_pin_B, right_wheel_pin, right_dir_pin, right_encoder_pin_A, right_encoder_pin_B):
        Device.pin_factory = PiGPIOFactory()
        
        self._left_wheel_pin = left_wheel_pin
        self._right_wheel_pin = right_wheel_pin
        self._compass = adafruit_lsm303dlh_mag.LSM303DLH_Mag(board.i2c())
        
        rotary_encoder.connect(
            clk_pin=left_encoder_pin_A,
            dt_pin=left_encoder_pin_B,
            on_clockwise_turn=self._encoder_increment,
            on_counter_clockwise_turn=self._encoder_decrement
            )
        rotary_encoder.connect(
            clk_pin=right_encoder_pin_A,
            dt_pin=right_encoder_pin_B,
            on_clockwise_turn=self._encoder_increment,
            on_counter_clockwise_turn=self._encoder_decrement
            )
        
        self.left_motor = PWM_pin(left_wheel_pin)
        self.left_motor_dir = D_pin(left_dir_pin)
        self.right_motor = PWM_pin(right_wheel_pin)
        self.right_motor_dir = D_pin(right_dir_pin)
        
        self._x_pos = 0
        self._y_pos = 0
        self.angle_offset = 0
        self.angle_offset = self.get_orientation()

            
    def go_to_position(self, x, y):
        distance = np.sqrt((x-self._x_pos)**2+(y-self._y_pos)**2)
        angle = np.arctan2(y-self._y_pos, x-self._x_pos)
        
        self._rotate_vehicle_to(angle)
        self.travel_distance_forward(distance)
        
        
    def _travel_distance_forward(self, distance):
        starting_x = self._x_pos
        starting_y = self._y_pos
        
        time_0 = time.time()
        speed = 0
        accel = self.acceleration
        speed_0 = 0
        
        remaining_distance = distance
        while(remaining_distance > 0):
            remaining_distance = distance - np.sqrt((self._x_pos-starting_x)**2+(self._y_pos-starting_y)**2)
            stopping_distance = speed**2/(2*self.acceleration)

            if remaining_distance > stopping_distance:
                speed = np.clip(speed_0 + (time.time()-time_0)*accel, 0, self.max_velocity)
                
            elif remaining_distance < stopping_distance and accel > 0:
                time_0 = time.time()
                speed_0 = speed
                accel = -self.acceleration
            
            
            self._set_left_motor_speed(speed)
            self._set_right_motor_speed(speed)
        
        self._set_left_motor_speed(0)
        self._set_right_motor_speed(0)
        
        
    def _rotate_vehicle_to(self, angle):
        
        time_0 = time.time()
        speed = 0
        accel = self.acceleration
        speed_0 = 0
        
        while(np.abs(angle - self.get_orientation()) > self.target_angle_tollerance):
            delta_angle = angle - self.get_orientation()
            direction = np.sign(delta_angle)
            delta_angle = abs(delta_angle)
            if delta_angle > np.pi:
                direction = -direction
                delta_angle -= np.pi
            
            remaining_distance = delta_angle*self.stationary_turn_radius
            stopping_distance = self.tangential_turning_velocity * speed**2/(2*self.acceleration)
            
            if remaining_distance > stopping_distance:
                speed = np.clip(speed_0 + (time.time()-time_0)*accel, 0, self.max_velocity)
                
            elif remaining_distance < stopping_distance and accel > 0:
                time_0 = time.time()
                speed_0 = speed
                accel = -self.acceleration
            
            self._set_left_motor_speed(speed*(-direction))
            self._set_right_motor_speed(speed*direction)
        self._set_left_motor_speed(0)
        self._set_right_motor_speed(0)
            
    def get_orientation(self):
        magnet_x, magnet_y, _ = self._compass.megnetic
        return np.arctan2(magnet_y, magnet_x)-self.angle_offset
        
    def _set_right_motor_speed(self, speed):
        self.right_motor_dir.value = speed < 0
        self.right_motor.value = self._speed_to_pwm(speed)
        
    def _set_left_motor_speed(self, speed):
        self.left_motor_dir.value = speed < 0
        self.left_motor.value = self._speed_to_pwm(speed)
    
    def _speed_to_pwm(self, speed):
        for i in range(0, len(self.speed_curve[0] - 1)):
            if speed >= self.speed_curve[0][i]:
                inter = (self.speed_curve[1][i+1] - self.speed_curve[i][i])*(speed-self.speed_curve[0][i])/(self.speed_curve[0][i]-self.speed_curve[0][i+1])
                return self.speed_curve[1][i] + inter
        raise ValueError("Speed outside available speed range.")
        
    def _encoder_increment(self):
        angle = self.get_orientation()
        self._x_pos += self.mm_per_step*np.sin(angle)/2
        self._y_pos += self.mm_per_step*np.cos(angle)/2
        
    def _encoder_decrement(self):
        angle = self.get_orientation()
        self._x_pos -= self.mm_per_step*np.sin(angle)/2
        self._y_pos -= self.mm_per_step*np.cos(angle)/2
