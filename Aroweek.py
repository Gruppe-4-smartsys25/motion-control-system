# -*- coding: utf-8 -*-
"""
Created on Mon Sep 15 00:21:36 2025

@author: Åsmund
"""

import numpy as np
import rotary_encoder
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
    

    def __init__(self, left_wheel_pin, left_encoder_pin_A, left_encoder_pin_B, right_wheel_pin, right_encoder_pin_A, right_encoder_pin_B):
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
        #figure out PWM
        print("not implemented")
        
    def _set_left_motor_speed(self, speed):
        #figure out PWM
        print("not implemented")
        
    def _encoder_increment(self):
        angle = self.get_orientation()
        self._x_pos += self.mm_per_step*np.sin(angle)/2
        self._y_pos += self.mm_per_step*np.cos(angle)/2
        
    def _encoder_decrement(self):
        angle = self.get_orientation()
        self._x_pos -= self.mm_per_step*np.sin(angle)/2
        self._y_pos -= self.mm_per_step*np.cos(angle)/2
