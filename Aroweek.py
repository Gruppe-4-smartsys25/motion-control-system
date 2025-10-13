# -*- coding: utf-8 -*-
"""
Created on Mon Sep 15 00:21:36 2025

@author: Potato
"""

import numpy as np
import Encoder
import adafruit_lsm303dlh_mag
import board

class Aroweek:
    wheel_diameter = 60 #mm
    steps_per_revolution = 4 #directly on motor
    motor_gear_ratio = 50/1 #50 to 1
    
    motor_speed = 10
    target_angle_tollerance = 0.01*2*np.pi #radian
    
    mm_per_step = (np.pi*wheel_diameter)/(steps_per_revolution*motor_gear_ratio)

    def __init__(self, left_wheel_pin, left_encoder_pin_A, left_encoder_pin_B, right_wheel_pin, right_encoder_pin_A, right_encoder_pin_B):
        self._left_wheel_pin = left_wheel_pin
        self._left_encoder = Encoder.Encoder(left_encoder_pin_A, left_encoder_pin_B)
        self._right_wheel_pin = right_wheel_pin
        self._right_encoder = Encoder.Encoder(right_encoder_pin_A, right_encoder_pin_B)
        self._compass = adafruit_lsm303dlh_mag.LSM303DLH_Mag(board.i2c())
        
        self._x_pos = 0
        self._y_pos = 0

            
    def go_to_position(self, x, y):
        distance = np.sqrt((x-self._x_pos)**2+(y-self._y_pos)**2)
        angle = np.arctan2(y-self._y_pos, x-self._x_pos)
        
        self._rotate_vehicle_to(angle)
        self.travel_distance_forward(distance)
        
        
    def _travel_distance_forward(self, distance):
        
        
    def _rotate_vehicle_to(self, angle):
        while(np.abs(angle - self.get_orientation()) > self.target_angle_tollerance):
            delta_angle = angle - self.get_orientation()
            direction = np.sign(delta_angle)
            if delta_angle > np.pi:
                direction = -direction
            self._set_left_motor_speed(self.motor_speed*(-direction))
            self._set_right_motor_speed(self.motor_speed*direction)
        self._set_left_motor_speed(0)
        self._set_right_motor_speed(0)
        
            
    def get_orientation(self):
        magnet_x, magnet_y, _ = self._compass.megnetic
        return np.arctan2(magnet_y, magnet_x)
        
    def _set_right_motor_speed(self, speed):
        #figure out PWM
        print("not implemented")
        
    def _set_left_motor_speed(self, speed):
        #figure out PWM
        print("not implemented")
        

    # Getter method (property)
    @property
    def mileage(self):
        return self._mileage

    # Setter method (property)
    @mileage.setter
    def mileage(self, value):
        if value >= self._mileage:  # can't roll back odometer
            self._mileage = value
        else:
            print("Mileage cannot be decreased!")

    # Class method
    @classmethod
    def change_wheels(cls, number):
        cls.wheels = number

    # Static method
    @staticmethod
    def honk():
        print("Beep beep!")

    # String representation
    def __str__(self):
        return f"{self.year} {self.brand} {self.model} with {self.wheels} wheels"
