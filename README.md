# LegoFerrari

ROS package for control the parking maneuvers of an autonomous car-like-problem.

Features:

1. Manual driving of a car-like robot throught wifi connection and a joystick

2. Simulation of car-like robot with a rear motor and a front steering wheel

3. Autonomous driving of a car-like robot to a goal pose


# Requirements

For running each sample code:

- [Python 3.10.x](https://www.python.org/)
- [NumPy](https://numpy.org/)
- [SciPy](https://scipy.org/)
- [Matplotlib](https://matplotlib.org/)
- [ROS joy](http://wiki.ros.org/joy)
- [ROS rosserial_server](http://wiki.ros.org/rosserial_server)
 

# How to install

1. Open the catkin_ws/src directory

> cd ~/catkin_ws/src

2. Clone this repo.

> git clone https://github.com/Matteo-99/LegoFerrari.git

3. Make the lego_ferrari package

> cd ~/catkin_ws
> catkin_make

3. Install the package dependancy.

initialize rosdep :

> sudo rosdep init
> rosdep update

install dependacy for the lego_ferrari package :

> rosdep install lego_ferrari

4. Re-build the catkin workspace

> cd ~/catkin_ws
> catkin_make

5. The installation is complete and the package is ready to be used.


# Contents

## Launch files

### autonomous_ferrari.launch
This launcher starts the /ps4_controller node to control the PS4 joystick, 

### test_simulate_car.launch

## Ros nodes

### joy_mux_navigation_node

### navigator_node

### saturate_cmd_node

### simulated_car_node

### test_node


# Authors

- [Matteo Sperti](https://github.com/Matteo-99)
- [Morgan Casale](https://github.com/morgancasale)
- Federico Moscato
- Francesco Stolcis
- Paolo Timis

# References

- [AtsushiSakai/PythonRobotics](https://arxiv.org/abs/1808.10703)
- B. Siciliano, L. S. (2007). Robotics. Modelling, planning and control. Springer.
- Corke, P. (2011). Robotics, vision and control. Springer.
