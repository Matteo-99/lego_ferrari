#!/usr/bin/env python3

""" 
DC motor model
 """

from TransferFunction import TransferFunction
import numpy as np

class DC_motor:
    """
    Constructs an instantiate of the DC_motor to simulate the dynamical behaviour of a DC motor
    """
    def __init__(self, ta, tm, kt, k_speed, ka, kv, T_static, u_r, beta_viscous, dt):
        self.dt = dt

        self.k_speed = k_speed
        self.ka = ka
        self.kv = kv
        
        self.beta_viscous = beta_viscous
        self.T_static = T_static                # Static torque
        self.T_dynamic = u_r * T_static         # Dynamic torque
        
        # voltage on the armature to Torque Transfer function--> kt/(1 + ta*s) in kt we consider also 1/Ra
        self.v2T = TransferFunction([kt], [ta, 1], self.dt)  
        # Resultant Torque to w Transfer function--> 1/beta/(1 + tm*s)           
        self.T2w = TransferFunction([1/beta_viscous], [tm, 1], self.dt) 

        self.is_moving = False
        self.w = 0.0

    def clear(self):
        self.v2T.clear()
        self.T2w.clear()
        self.w = 0.0
        self.is_moving = False

    def update(self, u):
        """
        Update the state of the motor according to the dynamical model and the input u.
        Returns the value of the current linear velocity
        Arguments
        ----------
        u : input voltage duty cycle value expressed on 8 bit unsigned integer
        Returns
        -------
        v : Linear velocity
        """
        if self.is_moving:
            T_load = self.T_dynamic

            Va = u*self.ka - self.kv*self.w
            Tm = self.v2T.update(Va)

            if Tm < 0:      
                T_load = 0.95 * T_load

            T_eff = Tm - T_load*np.sign(Tm)
            self.w = self.T2w.update(T_eff)
            if abs(self.w) < 1 and Tm*np.sign(Tm) < T_load:
                self.clear()

        else:
            T_load = self.T_static

            Va = u*self.ka - self.kv*self.w
            Tm = self.v2T.update(Va)

            if Tm*np.sign(Tm) > T_load:
                self.is_moving = True
                T_eff = Tm - T_load*np.sign(Tm)
                
            else:
                T_eff = 0.0
            
            self.w = self.T2w.update(T_eff)

        v = self.w * self.k_speed

        #print(Va, Tm, T_eff, self.w, v)
        return v