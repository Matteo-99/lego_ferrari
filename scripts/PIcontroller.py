#!/usr/bin/env python3

""""
PI Controller model.
"""

import numpy as np

class PIcontroller:
    """
    Constructs an instantiate of the PIcontroller
    Parameters
    ----------
    Kp : Proportional gain of the controller
    Ki : Integral gain of the controller
    dt : Integration step
    max : Maximum absolute value of the output
    """
    def __init__(self, kp, ki, dt, max = 100000):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.max = max
        self.current = 0.0
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

        if abs(y) > self.max:
            y = np.sign(y) * self.max

        return y
