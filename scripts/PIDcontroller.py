#!/usr/bin/env python3

""""
PID Controller model.
In this first version only the proportional part is present, if necessary it is possible to add the 
integro-derivative part
"""

import numpy as np
from TransferFunction import TransferFunction

class PIDcontroller:
    def __init__(self, kp, ki, dt, max = 100000, start_offset = 0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.max = max
        self.current = 0.0
        self.start_offset = start_offset
        self.i_err = 0.0

    def set_current(self, v):
        self.current = v

    def clear(self):
        self.current = 0.0
        self.i_err = 0.0

    def calc_control(self, target):
        e = target - self.current
        self.i_err += e * self.dt
        y = self.kp * e + self.ki*self.i_err
        if abs(self.current) < 0.0001 * self.max:
            y = y + self.start_offset*np.sign(e)

        if abs(y) > self.max:
           y = np.sign(y) * self.max

        return y
