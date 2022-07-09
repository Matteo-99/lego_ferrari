#!/usr/bin/env python3

""" 
This scripts simulate the behaviour of the car-like robot. 
It received as input a <Ferrari_command> msg and send the actual pose as output.
As the real car-like robot, it first saturates the input command and then it tries to move 
accordingly to it.
In order to model this custom car-like robot the classical bycicle model has been properly adapted.
 """

import rospy
import numpy as np
from CmdSaturation import CmdSaturation
from CarModel import BicycleModel
from lego_ferrari.msg import Ferrari_command
from lego_ferrari.msg import State
from std_msgs.msg import Bool
       

def callback_cmd(cmd_received):
    # Saturate the input command in the same way of the real car-like robot
    cmd_sat = sat.update_cmd(cmd_received)
    print(cmd_sat)
    # Set the actual command to the simulated car
    my_car.set_cmd(cmd_sat)  

# Reinitialization of the simulated car
def callback_clear(data):
    if data:
        my_car.clear()
        sat.clear()

if __name__ == '__main__':
    try:
        rospy.init_node('simulated_car')
        #command limits
        cmd_max_velocity = rospy.get_param("/cmd_max_velocity", 100) 
        cmd_max_angle = rospy.get_param("/cmd_max_angle", 70)
        max_turn = rospy.get_param("/cmd_max_turn", 35)
        max_acc_step = rospy.get_param("/cmd_max_acc_step", 2)
        start_acc_step = rospy.get_param("/cmd_start_acc_step", 20)
        max_dec_step = rospy.get_param("/cmd_max_dec_step", 4)
        cmd_zero_tol = rospy.get_param("/cmd_zero_speed_tol", 3)
        brake_step = rospy.get_param("/inertia_stop_step", 10)
        
        #physics limit
        max_psi = np.deg2rad(rospy.get_param("/max_psi", 21))
        wheelbase = rospy.get_param("/wheelbase", 0.37)
        T_static = rospy.get_param("/T_load_static", 1)
        beta_viscous = rospy.get_param("/beta_viscous", 0.5)
        u_r = rospy.get_param("/u_r_motor", 0.5)
        ta = rospy.get_param("/ta_motor", 0.5) 
        kt = rospy.get_param("/kt_motor", 0.5)
        tm = rospy.get_param("/tm_motor", 0.5) 
        k_speed = rospy.get_param("/k_speed_motor", 0.5) 
        ka = rospy.get_param("/ka_motor", 0.5) 
        kv = rospy.get_param("/kv_motor", 0.5)

        repeat = rospy.get_param("/repeat_rate", 10.0)
        dt = 1/repeat
        pub = rospy.Publisher('simulated_car/State', State, queue_size=1)
        sub = rospy.Subscriber('simulated_car/cmd_vel', Ferrari_command, callback_cmd)
        sub_clear = rospy.Subscriber('clear_nav', Bool, callback_clear)
        rate = rospy.Rate(repeat) # default 10hz

        # object that saturates the command in input to the simulated car
        # (it is equal to the one that saturate the input command to the real car)
        sat = CmdSaturation(cmd_max_velocity, max_acc_step, start_acc_step, max_dec_step, 
                                cmd_zero_tol, brake_step, cmd_max_angle, max_turn)
        
        # Object that simulate the behaviour of the car
        my_car = BicycleModel(wheelbase, cmd_max_angle, max_psi, ta, tm, kt, k_speed, ka, kv, 
                                T_static, u_r, beta_viscous, dt)


        while not rospy.is_shutdown():
            ActualState = my_car.move()        
            pub.publish(ActualState)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
