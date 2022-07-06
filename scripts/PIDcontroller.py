#!/usr/bin/env python3

""""
PID Controller model.
In this first version only the proportional part is present, if necessary it is possible to add the 
integro-derivative part
"""

import numpy as np

class PIDcontroller:
    def __init__(self,kp, max = 100000, start_offset = 0):
        self.k = kp
        self.max = max
        self.current = 0.0
        self.start_offset = start_offset

    def set_current(self, v):
        self.current = v

    def clear(self):
        self.current = 0.0

    def calc_control(self, target):
        e = target - self.current
        #print(self.current, target, e) 
        if abs(self.current) < 0.0001 * self.max:
            y = self.k * e + self.start_offset*np.sign(e)
        else:
            y = self.k * e 

        if abs(y) > self.max:
           y = np.sign(y) * self.max

        return y
