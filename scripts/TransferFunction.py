#!/usr/bin/env python3

"""
Compute the output of a given transfer function
"""

import scipy.signal
import numpy

class TransferFunction:
    def __init__(self, num, den, Ts, points = 1, tol = 0.00001):
        self.dt = Ts/float(points)
        self.points = points
        self.tol = tol

        G = scipy.signal.lti(num, den)
        self.Gss = G.to_ss()
        self.x = numpy.zeros((self.Gss.A.shape[0], 1))

    def clear(self):
        self.x = numpy.zeros((self.Gss.A.shape[0], 1))
    
    def update(self, u):
        for _ in range(self.points):
            xdot = self.Gss.A.dot(self.x) + self.Gss.B.dot(u)
            y = self.Gss.C.dot(self.x) +  self.Gss.D.dot(u)
            self.x += xdot*self.dt

            if numpy.linalg.norm(self.x) < self.tol and numpy.linalg.norm(xdot) < self.tol:
                self.clear()

        return y[0,0]