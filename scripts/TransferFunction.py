#!/usr/bin/env python3

"""
Compute the output of a given transfer function
"""

import scipy.signal
import numpy

class TransferFunction:
    """
    Constructs an instantiate of the TransferFunction for compute the
    Parameters
    ----------
    num : Vector of the coefficient of the numerator of the TF 
    den : Vector of the coefficient of the denominator of the TF 
    Ts : Integration step
    tol : Zero tolerance (default value is 10^-5)
    """
    def __init__(self, num, den, Ts, tol = 0.00001):
        self.dt = Ts
        self.tol = tol

        G = scipy.signal.lti(num, den)
        self.Gss = G.to_ss()
        self.x = numpy.zeros((self.Gss.A.shape[0], 1))

    def clear(self):
        self.x = numpy.zeros((self.Gss.A.shape[0], 1))
    
    def update(self, u):
        """
        Returns the current output of the TF according to the input u
        """
        xdot = self.Gss.A.dot(self.x) + self.Gss.B.dot(u)
        y = self.Gss.C.dot(self.x) +  self.Gss.D.dot(u)
        self.x += xdot*self.dt

        if numpy.linalg.norm(self.x) < self.tol and numpy.linalg.norm(xdot) < self.tol:
            self.clear()

        return y[0,0]