#!/usr/bin/env python3

""""
This script is used for navigating a car-like robot on a 2D plane.
It receives as input the Goal Pose, while the Start Pose = [0 0 0] by defalt.

1) compute the path using the <path_planner> functions
2) navigates the robot to the serie of goal in order to arrive to the final Goal.
    For each goal it exploit the move2goal control solution of: 
    P. I. Corke, "Robotics, Vision & Control", Springer 2017
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import path_planner as path_planner
from PathFinderController import PathFinderController
from PIcontroller import PIcontroller
from geometry_msgs.msg import Pose2D
from lego_ferrari.msg import Ferrari_command
from lego_ferrari.msg import State
from std_msgs.msg import Bool
import rospy

class Move2Goal:
    def __init__(self, tolerance, cmd_max_linear_speed, cmd_max_angle, max_velocity, 
                    max_psi, wheelbase, curvature, show_animation, dt, ax_path, ax_traj):
        self.tolerance = tolerance                          # tolerance for the goal
        self.cmd_max_linear_speed = cmd_max_linear_speed    # max linear speed command accepted from the control
        self.cmd_max_angle = cmd_max_angle                  # max angle accepted from the control
        self.max_speed = max_velocity                       # max linear speed
        self.max_angle = max_psi                            # max steering angle
        self.wheelbase = wheelbase                          # distance between the two axis of wheels
        self.curvature = curvature                          # min curvature of the vehicle

        self.goal_vector = []
        self.ActualGoal = Pose2D()
        self.StartPose = Pose2D()
        self.ActualPose = self.StartPose       
        self.v = 0.0

        self.x_traj = [self.ActualPose.x]
        self.y_traj = [self.ActualPose.y]
        self.theta_traj = [self.ActualPose.theta]
        
        self.reached = False
        self.dt = dt
        self.run = False
        self.index  = 0

        self.show_animation = show_animation
        self.ax_path = ax_path
        self.ax_traj = ax_traj
        
    def clear(self): # initialize pose and goal vector to deafult values
        self.ActualGoal.x = 0.0
        self.ActualGoal.y = 0.0
        self.ActualGoal.theta = 0.0
        self.StartPose.x = 0.0
        self.StartPose.y = 0.0
        self.StartPose.theta = 0.0
        self.ActualPose = self.StartPose
        self.x_traj = [self.ActualPose.x]
        self.y_traj = [self.ActualPose.y]
        self.theta_traj = [self.ActualPose.theta]
        self.reached = False
        self.goal_vector = []
        self.run = False
        self.index  = 0
        self.ax_path.clear()
        self.v = 0.0
        
    def new_goal(self, GoalPose):   
        self.goal_vector = path_planner.planner(self.StartPose, GoalPose, self.wheelbase, 
                                                 self.curvature, self.dt, self.ax_path, )
        # get goal vector

        if not self.goal_vector: # checks if the goal vector is been created
            pub_goal_reached.publish(False)
        else:
            print("Path founded\n")
            self.run = True
    
    # move the robot to the goal           
    def move(self): 
        Cmd_vel = Ferrari_command()
        self.state_update()                 # updates the goal
        v, psi = self.calc_control()        # calculates velocity and servo angle to get to current goal

        Cmd_vel.linear_velocity = int(v)    # the car accepts only integer values for the velocity
        Cmd_vel.servo = int(psi)            # the car accepts only integer values for the servo angle
        Cmd_vel.brake = 0    
        pub_sim.publish(Cmd_vel)            # publish move command to the simulated car node
        pub_navigator.publish(Cmd_vel)      # publish move command to the navigator node of the real car
    
    # changes the goal if the car hasn't reached the last one
    def state_update(self): 
        if self.reached and self.v < 0.1:
            self.reached = False
            self.index = self.index+1
            PI_vel.clear()
            PI_psi.clear()
            if self.index >= len(self.goal_vector):
                pub_goal_reached.publish(True)    
                self.run = False
        if self.run:
            goal = self.goal_vector[self.index] # get the next goal
            self.ActualGoal.x = goal[0]
            self.ActualGoal.y = goal[1]
            self.ActualGoal.theta = goal[2]      

    def setActualState(self, data):
        ActualPose = Pose2D(data.x, data.y, data.theta)
        self.ActualPose = ActualPose
        PI_vel.set_current(data.v)      # set the current velocity for the PI controller
        PI_psi.set_current(data.psi)    # set the current steering angle for the PI controller
        self.v = data.v
        if (abs(ActualPose.x -  self.x_traj[-1]) > 0.001) or (abs(ActualPose.y -  self.y_traj[-1]) > 0.001):     
            self.x_traj.append(self.ActualPose.x)
            self.y_traj.append(self.ActualPose.y)
            self.theta_traj.append(self.ActualPose.theta)
        ''' if the car position is different 
            from the previous one (the car moved),
            appends the actual last position to the trajectory'''
   
    # calculates the control law to get to the next goal
    def calc_control(self): 
        x = self.ActualPose.x
        y = self.ActualPose.y
        theta = self.ActualPose.theta
        
        x_goal = self.ActualGoal.x
        y_goal = self.ActualGoal.y
        theta_goal = self.ActualGoal.theta
        
        if  self.run:
            x_diff = x_goal - x
            y_diff = y_goal - y

            rho = np.hypot(x_diff, y_diff) # distance from the goal

            if rho > self.tolerance: # if the car distance is not in the range of tolerance of the goal     
                v, w = controller.calc_control_command(x_diff, y_diff, theta, theta_goal) # find velocity and angular velocity
                if(abs(v) > 0):
                    psi = np.arctan(w*self.wheelbase/abs(v))    # calculate the steering angle in rad
                else:
                    psi = np.arctan(w*self.wheelbase/0.00001)
                if abs(psi) > self.max_angle:               # security check for the steering angle
                    psi = np.sign(psi) * self.max_angle
                psi = psi*(180/np.pi)                       # conversion to deg                
                cmd_psi = PI_psi.calc_control(psi)

                if abs(v) > self.max_speed: #security check for the motor speed
                    v = np.sign(v) * self.max_speed

                cmd_v = PI_vel.calc_control(v)
                print(v, psi)
                return cmd_v, cmd_psi
            else :
                self.reached = True
                return 0.0, 0.0
        else:
            return 0.0, 0.0

    def show(self):
            if self.show_animation:
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

                self.ax_traj.arrow(self.StartPose.x, self.StartPose.y, 
                        self.wheelbase * np.cos(self.StartPose.theta),
                        self.wheelbase * np.sin(self.StartPose.theta), fc='r',
                        head_width=0.5*self.wheelbase, 
                        head_length=0.5*self.wheelbase, length_includes_head = True)
                self.ax_traj.arrow(self.ActualGoal.x, self.ActualGoal.y, 
                        self.wheelbase * np.cos(self.ActualGoal.theta), 
                        self.wheelbase * np.sin(self.ActualGoal.theta), fc='g',
                        head_width=0.5*self.wheelbase, 
                        head_length=0.5*self.wheelbase, length_includes_head = True)

                self.ax_traj.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
                self.ax_traj.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
                self.ax_traj.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

                self.ax_traj.plot(self.x_traj, self.y_traj, 'b--')
                
                self.ax_traj.grid()
                x_min, x_max = self.ax_path.get_xlim()
                y_min, y_max = self.ax_path.get_ylim()
                self.ax_traj.set_xlim([x_min- 1, x_max +1])
                self.ax_traj.set_ylim([y_min- 1, y_max +1])
        

def transformation_matrix(x, y, theta): # 2D transformation matrix
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta),  y],
        [0,             0,              1]
    ])
        
def callback_pose(data):
    nav.setActualState(data)

def callback_clear(data):
    if data:
        nav.clear()
        PI_vel.clear()
        PI_psi.clear()

def callback_new_goal(data):
    nav.new_goal(data)    
    

def navigate(i):
    if not rospy.is_shutdown():
        nav.move()
        nav.show()
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('navigator')

        # simulation parameters
        tolerance = rospy.get_param("/tolerance", 0.3) 
        repeat = rospy.get_param("/repeat_rate", 10.0)
        dt = 1/repeat
        msec =  int(1000/repeat)
        show_animation = rospy.get_param("/show_animation", True)

        # command limits
        cmd_max_velocity = rospy.get_param("/cmd_max_velocity", 100) 
        cmd_max_angle = rospy.get_param("/cmd_max_angle", 70)
        cmd_min_move = rospy.get_param("/cmd_min_move", 30)
        
        # physics limit
        wheelbase = rospy.get_param("/wheelbase", 0.37)
        curvature = 1/rospy.get_param("/curvature", 1)
        max_velocity = rospy.get_param("/max_speed", 3)
        max_psi = np.deg2rad(rospy.get_param("/max_psi", 21))

        # controller parameters
        k_rho = rospy.get_param("/k_rho", 9)
        k_alpha = rospy.get_param("/k_alpha", 15)
        k_beta = rospy.get_param("/k_beta", 3)
        controller = PathFinderController(k_rho, k_alpha, k_beta)

        kp_vel = rospy.get_param("/kp_vel", 30)
        ki_vel = rospy.get_param("/ki_vel", 30)
        PI_vel = PIcontroller(kp_vel, ki_vel, dt, cmd_max_velocity, cmd_min_move)

        kp_psi = rospy.get_param("/kp_psi", 30)
        ki_psi = rospy.get_param("/ki_psi", 30)
        PI_psi = PIcontroller(kp_psi, ki_psi, dt, cmd_max_angle)
    	
        fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1) # create figure and subplots for simulation
        nav = Move2Goal(tolerance, cmd_max_velocity, cmd_max_angle, max_velocity, 
                            max_psi, wheelbase, curvature, show_animation, dt, ax1, ax2)
        
        #initialization of nodes
        pub_sim = rospy.Publisher('simulated_car/cmd_vel', Ferrari_command, queue_size=1)
        pub_navigator = rospy.Publisher('navigator/cmd_vel', Ferrari_command, queue_size=1)
        pub_goal_reached = rospy.Publisher('navigator/goal_reached', Bool, queue_size=1)
        
        sub = rospy.Subscriber('simulated_car/State', State, callback_pose)
        sub_goal = rospy.Subscriber('navigator/goal', Pose2D, callback_new_goal)
        sub_stopNav = rospy.Subscriber('clear_nav', Bool, callback_clear)
        
        rate = rospy.Rate(repeat) # 10hz

        ani = animation.FuncAnimation(fig, navigate, interval=0.7*msec)
        plt.show()

    except rospy.ROSInterruptException:
        plt.close('all')
        pass
