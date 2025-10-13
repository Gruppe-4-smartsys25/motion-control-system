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
    steps_per_revolution = 200
    
    mm_per_step = (np.pi*wheel_diameter)/steps_per_revolution

    def __init__(self, left_wheel_pin, left_encoder_pin_A, left_encoder_pin_B, right_wheel_pin, right_encoder_pin_A, right_encoder_pin_B):
        self.left_wheel_pin = left_wheel_pin
        self.left_encoder = Encoder.Encoder(left_encoder_pin_A, left_encoder_pin_B)
        self.right_wheel_pin = right_wheel_pin
        self.right_encoder = Encoder.Encoder(right_encoder_pin_A, right_encoder_pin_B)
        self.compass = adafruit_lsm303dlh_mag.LSM303DLH_Mag(board.i2c())
        
        self.x_pos = 0
        self.y_pos = 0

    # Instance method
    def drive(self, miles):
        """Increase mileage when the car is driven."""
        if miles > 0:
            self._mileage += miles
            print(f"{self.brand} {self.model} drove {miles} miles.")
        else:
            print("Miles must be positive!")
            
    def get_orientation(self):
        magnet_x, magnet_y, _ = self.compass.megnetic
        return np.arctan2(magnet_y, magnet_x)

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
