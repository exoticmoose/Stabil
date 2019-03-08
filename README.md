## Stabil

Catkin workspace for Senior Design project, built for ROS primarily in C++

# Overview
Self-stabilizing robot actuated by four servo motors controling legs, with DC drive motors attached at the end.
Primary features are joystick attitude control, calculated leg positioning for ground plane contact, non-planar ground compensation, 
wheel torque request vectoring, and computer vision (external nodes) control integration.

Uses a handful of libraries:
- JHPWMPCA9685 used to drive PCA9685 PWM generator over I2C from Jetson TX2
- RTIMULIB used to access BNO055 over I2C
- PID used for X/Y chassis stabilization (Does not need to be compiled here, just custom launch config)
