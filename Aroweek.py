# -*- coding: utf-8 -*-
"""
Created on Mon Sep 15 00:21:36 2025

@author: Potato
"""

class Aroweek:
    # Class attribute (shared by all instances)
    

    def __init__(self, wheelsL, wheelsR):
        # Instance attributes (unique to each object)
        self.wheelsL = wheelsL
        self.wheelsR = WheelsR

    # Instance method
    def drive(self, miles):
        """Increase mileage when the car is driven."""
        if miles > 0:
            self._mileage += miles
            print(f"{self.brand} {self.model} drove {miles} miles.")
        else:
            print("Miles must be positive!")

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
