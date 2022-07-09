#!/usr/bin/env python3

""" 
This scripts is used with the RobotSteering tool of rqt in order to test the car parameter. The input topic is /test_cmd
 """

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from lego_ferrari.msg import Ferrari_command
from lego_ferrari.msg import State
from geometry_msgs.msg import Twist

class TestClass:
    def __init__(self, ax, wheelbase):
        self.cmd_sent = Ferrari_command()
        self.cmd_sent.brake = 0
        self.cmd_sent.servo = 0
        self.ax_traj = ax
        self.x_traj = [0.0]
        self.y_traj = [0.0]
        self.theta_traj = [0.0]
        self.wheelbase = wheelbase

    def set_cmd(self, speed):
        self.cmd_sent.linear_velocity = int(10*speed)

    def get_cmd(self):
        return self.cmd_sent

    def show(self):
        self.ax_traj.clear()

        x = self.x_traj[-1]
        y = self.y_traj[-1]
        theta = self.theta_traj[-1]

        # Corners of triangular vehicle when pointing to the right (0 rad) in the form [x,y,1].T
        p1_i = np.array([0.9*self.wheelbase, 0*self.wheelbase, 1]).T
        p2_i = np.array([-0.1*self.wheelbase, 0.25*self.wheelbase, 1]).T
        p3_i = np.array([-0.1*self.wheelbase, -0.25*self.wheelbase, 1]).T

        T = transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)


        self.ax_traj.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        self.ax_traj.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        self.ax_traj.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        self.ax_traj.plot(self.x_traj, self.y_traj, 'b--')
        self.ax_traj.set_xlim([-1, 10])
        self.ax_traj.set_ylim([-1, 1])
        self.ax_traj.grid()

    def setActualState(self, data):
        if (abs(data.x -  self.x_traj[-1]) > 0.001) or (abs(data.y -  self.y_traj[-1]) > 0.001):       
            self.x_traj.append(data.x)
            self.y_traj.append(data.y)
            self.theta_traj.append(data.theta)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def callback_cmd(cmd_received):
    test.set_cmd(cmd_received.linear.x)

def callback_pose(data):
    test.setActualState(data)
    #print(data.v)

def run(i):
    if not rospy.is_shutdown():
        cmd_sent = Ferrari_command()
        cmd_sent = test.get_cmd()
        pub.publish(cmd_sent)
        test.show()
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('test_cmd')
        repeat = rospy.get_param("/repeat_rate", 10.0)
        msec =  int(1000/repeat)
        wheelbase = rospy.get_param("/wheelbase", 0.37)

        pub = rospy.Publisher('simulated_car/cmd_vel', Ferrari_command, queue_size=1)
        sub_cmd = rospy.Subscriber('test_cmd', Twist, callback_cmd)
        sub = rospy.Subscriber('simulated_car/State', State, callback_pose)

        rate = rospy.Rate(repeat) # default 10hz     

        fig, ax = plt.subplots()
        test = TestClass(ax, wheelbase)
        ani = animation.FuncAnimation(fig, run, interval=msec)
        plt.show()
    except rospy.ROSInterruptException:
        pass