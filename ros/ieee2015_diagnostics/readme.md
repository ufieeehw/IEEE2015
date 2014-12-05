Robot Diagnostics
=================


This package contains ROS tools for debugging the robot, automating out some of the tedium involved in test/debug.


# Utilities

Arm click-control
    roslaunch ieee2015_diagnostics arm_click_control.launch

Arm state visualization (Includes torques, angles, velocities)
    rosrun ieee2015_diagnostics visualize_arm_behavior topic='/robot/elbow_controller'

Magnetometer visualization (Includes X,Y Mag)
    rosrun ieee2015_diagnostics viualize_imu_output topic='/robot/imu'


## TODO

- Visualize angle-error vs. time for all servos
- Visualize position with printed angles
- Visualize target angles and error
- Visualize torque-vs-angle
- Send a sequence of test instructions
- Send a user-inputted location instruction
- Test servo PID
- Test send-to-position with a calibration view

### Plots
- Make a unified plotting package for plotting ROS stuff....RVIZ?