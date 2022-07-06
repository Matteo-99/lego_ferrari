#!/usr/bin/env python3

"""
This script uses the <CmdSaturation> class in order to safely saturate the command that are sent
to the car-like robot
"""

import rospy
import CmdSaturation as CmdSat
from lego_ferrari.msg import Ferrari_command
       
def callback_cmd(cmd_received):
    cmd_sent = Ferrari_command()
    cmd_sent = sat.update_cmd(cmd_received)    
    pub_2LegoFerrari.publish(cmd_sent)
    
if __name__ == '__main__':
    rospy.init_node('saturate_cmd')

    max_velocity = rospy.get_param("/cmd_max_velocity", 150)
    max_angle = rospy.get_param("/cmd_max_angle", 70)
    max_turn = rospy.get_param("/cmd_max_turn", 35)
    max_acc_step = rospy.get_param("/cmd_max_acc_step", 2)
    start_acc_step = rospy.get_param("/cmd_start_acc_step", 20)
    max_dec_step = rospy.get_param("/cmd_max_dec_step", 4)
    cmd_zero_tol = rospy.get_param("/cmd_zero_speed_tol", 3)
    brake_step = rospy.get_param("/inertia_stop_step", 10)
    repeat = rospy.get_param("/repeat_rate", 10.0)
	
    sat = CmdSat.CmdSaturation(max_velocity, max_acc_step, start_acc_step, max_dec_step, 
                                    cmd_zero_tol, brake_step, max_angle, max_turn)

    pub_2LegoFerrari = rospy.Publisher('LegoFerrari_cmd', Ferrari_command, queue_size=1)
    sub_cmd = rospy.Subscriber('cmd_vel', Ferrari_command, callback_cmd)

    rospy.spin()
