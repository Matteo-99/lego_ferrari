#!/usr/bin/env python3

"""
This class saturates the command accordingly to the electrical and mechanical charateristic of
the real car-like model in order to garantee a safe behaviour.

At each iteration both the velocity and the sterring angle are saturated if necessary.
Moreover also the difference step are saturated. Different limits are used while accelerating
decelerating, starting to move or stopping by inertia
"""

from lego_ferrari.msg import Ferrari_command

class CmdSaturation:
    def __init__(self, max_velocity, max_acc_step, start_acc_step, max_dec_step, 
                    cmd_zero_tol, brake_step, max_angle, max_turn):
        self.cmd = Ferrari_command()
        self.max_velocity = max_velocity
        self.max_acc_step = max_acc_step
        self.start_acc_step = start_acc_step
        self.max_dec_step = max_dec_step
        self.cmd_zero_tol = cmd_zero_tol
        self.brake_step = brake_step
        self.max_angle = max_angle
        self.max_turn = max_turn

    def clear(self):
        self.cmd.brake = 0
        self.cmd.linear_velocity = 0
        self.cmd.servo = 0

    def get_cmd(self):
        return self.cmd

    def update_cmd(self, cmd_received):
        if cmd_received.brake:
            # stopping by inertia
            self.cmd.brake = 1

            # the velocity is artificially reduced (we have not sensor
            # on the real inertia of the car-like robot)
            if (abs(self.cmd.linear_velocity) > self.brake_step):
                self.cmd.linear_velocity -= self.brake_step * sign(self.cmd.linear_velocity)
            else:
                self.cmd.linear_velocity = 0
        else:
            self.cmd.brake = 0
            diff = cmd_received.linear_velocity - self.cmd.linear_velocity
            
            if (diff * sign(self.cmd.linear_velocity) >= 0) :   
                # Accellerating
                if abs(self.cmd.linear_velocity) <= self.cmd_zero_tol:
                    # Starting acceleration
                    diff = saturate(diff, self.start_acc_step)
                else:
                    diff = saturate(diff, self.max_acc_step)
                self.cmd.linear_velocity += diff
                self.cmd.linear_velocity = saturate(self.cmd.linear_velocity, self.max_velocity)
            else :  
                # Decelerating                                           
                diff = saturate(diff, self.max_dec_step)
                self.cmd.linear_velocity += diff
                if abs(self.cmd.linear_velocity) <= self.cmd_zero_tol:
                    self.cmd.linear_velocity = 0
        
        diff_servo = cmd_received.servo - self.cmd.servo
        diff_servo = saturate(diff_servo, self.max_turn)
        self.cmd.servo += diff_servo
        self.cmd.servo = saturate(self.cmd.servo, self.max_angle)
        
        return (self.cmd)

def saturate(data, maximum):
    if (data > maximum):
       data = maximum
    elif (data < -maximum):
       data =  -maximum
    return data
    
def sign(data):
    if (data > 0):
       return 1
    elif (data <0):
        return -1
    return 0