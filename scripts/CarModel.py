#!/usr/bin/env python3

""" 
In order to model this custom car-like robot the classical bycicle model has been properly adapted.
 """

import numpy as np
from DCmotorModel import DC_motor
from lego_ferrari.msg import State
from geometry_msgs.msg import Pose2D
from lego_ferrari.msg import Ferrari_command

       
class BicycleModel:
    def __init__(self, wheelbase, cmd_max_angle, max_psi, ta, kt, J, k_speed, ka, kv, T_static, u_r, beta_viscous, dt, points = 100):
        self.cmd_max_angle = cmd_max_angle
        self.max_psi = max_psi
        self.points = points
        self.Pose = Pose2D()
        self.cmd = Ferrari_command()
        self.wheelbase = wheelbase
        self.dt = dt/points
        self.motor = DC_motor(ta, kt, J, k_speed, ka, kv, T_static, u_r, beta_viscous, self.dt)

    # Bycicle model    
    def move(self):  
        for _ in range(self.points):
            psi = self.servo_update()

            v = self.motor.update(self.cmd.linear_velocity) # simulate DC motor to get a more precise linear velocity

            self.Pose.theta = self.Pose.theta + v*np.tan(psi)/self.wheelbase * self.dt # calculate current theta from the linear velocity and the servo angle
            self.Pose.x = self.Pose.x + v * np.cos(self.Pose.theta) * self.dt # calculate current x from the linear velocity and the current theta
            self.Pose.y = self.Pose.y + v * np.sin(self.Pose.theta) * self.dt # calculate current y from the linear velocity and the current theta

        return State(self.Pose.x, self.Pose.y, self.Pose.theta, v, psi)
    
    def clear(self):
        self.Pose.x = 0.0
        self.Pose.y = 0.0
        self.Pose.theta = 0.0
        self.motor.clear()
        self.cmd.linear_velocity = 0
        self.cmd.servo = 0
        self.cmd.brake = 0

    def set_cmd(self, cmd_received):
        self.cmd = cmd_received

    # This function convert the input <Ferrari_command.servo> msg into the real steering angle psi of the front wheels
    def servo_update(self):
        return self.cmd.servo/self.cmd_max_angle*self.max_psi
         