#!/usr/bin/env python3

"""
This script is used as controller of the manual/autonomous guide of the car-like robot.
It exploits the joy ros package and it uses a PS4 controller.

For what concerne the manual guide it converts the joystick signal into the <Ferrari_command> msg.
The conversion is simply linear

By pushing the triangle button on the joystick it is possible to set up the autonomous navigator.
The goal pose is inserted by the user by keyboard.

In any moment of the autonomous navigation it is possible to switch back to manual guide by pushing
the X button on the joystick.
"""

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from lego_ferrari.msg import Ferrari_command
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# joystick button config
FORWARD = 5	    # R2
BACKWARD = 2	# L2
ANGULAR = 0	    # left handle
BRAKE = 1	    # circle button
AUTONOMOUS = 2	# triangle button
MANUAL = 0	    # x button

cmd_joy = Ferrari_command()
cmd_navigator = Ferrari_command()
ready = False
autonomous_drive = False
goal_pose = Pose2D()
   
def is_ready(data):
    global ready
    if ready:
        return True
    elif (data.axes[FORWARD] == 1 and data.axes[BACKWARD] == 1):
        ready = True
        return True
    else:
        return False

class Manual(Exception):
    pass
    
def callback_joy(data):
    global autonomous_drive
    global cmd_joy
    global goal_pose
    
    if is_ready(data):
       joy2FerrariCommand(cmd_joy,data)
       try:       
           if data.buttons[MANUAL]:
               if autonomous_drive:
                   raise Manual
           elif data.buttons[AUTONOMOUS]:
               if not autonomous_drive:
                   pub_stopNav.publish(True)
                   
                   # Get the input from the user.
                   goal_pose.x=float(input("\n\nAutomatic pilot\nSet your x goal [m]: "))
                   goal_pose.y=float(input("Set your y goal [m]: "))
                   goal_pose.theta=float(input("Set your theta goal [deg]: "))
                   goal_pose.theta = np.deg2rad(goal_pose.theta)    #[rad]
                   pub_goal.publish(goal_pose)
                   autonomous_drive = True
       except Manual:
           print("\nNavigator aborted, back to manual guide\n")
           pub_stopNav.publish(True)
           autonomous_drive = False
           goal_pose.x=0
           goal_pose.y=0
           goal_pose.theta=0
        
def callback_navigator(data):
    global cmd_navigator
    cmd_navigator.servo = data.servo
    cmd_navigator.brake = 0
    cmd_navigator.linear_velocity = data.linear_velocity
        
def goal_reached(data):
    global goal_pose
    global autonomous_drive
    
    if data.data and autonomous_drive:
        print('Arrived at x: ' + repr(goal_pose.x) + ', y: ' + repr(goal_pose.y) + ', theta: ' + repr(goal_pose.theta) + '\nBack to Manual Guide \n')
        autonomous_drive = False
        #pub_stopNav.publish(True)
    if autonomous_drive and not data.data:
        print('Failed to arrived at x: ' + repr(goal_pose.x) + ', y: ' + repr(goal_pose.y) + ', theta: ' + repr(goal_pose.theta) + '\nNO PATH FOUNDED\nBack to Manual Guide \n')
        autonomous_drive = False
        pub_stopNav.publish(True)

def joy2FerrariCommand(cmd, data):
    cmd.servo = int(max_angle*data.axes[ANGULAR])
    cmd.brake = 0
    if data.buttons[BRAKE]:
        cmd.brake = 1
        cmd.linear_velocity = 0
    elif data.axes[FORWARD] != 1:
        cmd.linear_velocity = int((max_velocity*(1-data.axes[FORWARD])/2))
    elif data.axes[BACKWARD] != 1:
        cmd.linear_velocity = -int((max_velocity*(1-data.axes[BACKWARD])/2))
    else:
        cmd.linear_velocity = 0
       
    
def talker():
    global cmd_joy
    global cmd_navigator
    global autonomous_drive
    
    while not rospy.is_shutdown():   
        if autonomous_drive:
            pub_2LegoFerrari.publish(cmd_navigator)
        else:
            pub_2LegoFerrari.publish(cmd_joy)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mux_joy_navigator')

    max_velocity = rospy.get_param("/cmd_max_velocity", 150)
    max_angle = rospy.get_param("/cmd_max_angle", 70)
    repeat = rospy.get_param("/repeat_rate", 5.0)
	
    pub_2LegoFerrari = rospy.Publisher('cmd_vel', Ferrari_command, queue_size=1)
    pub_goal = rospy.Publisher('navigator/goal', Pose2D, queue_size=1)
    pub_stopNav = rospy.Publisher('clear_nav', Bool, queue_size=1)
    sub_joy = rospy.Subscriber('joy', Joy, callback_joy)
    sub_navigator = rospy.Subscriber('navigator/cmd_vel', Ferrari_command, callback_navigator)
    sub_goal_reached = rospy.Subscriber('navigator/goal_reached', Bool, goal_reached)

    rate = rospy.Rate(repeat) # default is 5hz
    
    print("\n\nManual guide\n")
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
