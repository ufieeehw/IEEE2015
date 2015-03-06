Maverick Controllers
===================

Nodes that mid and high-level control for Maverick. If you're curious about implementation, they are well-commented. For questions, check the git-blame for who to ask.


# Arm Controller
Given a desired position in space, computes the servo angles to achieve that position

## Usage
    rosrun ieee2015_controller arm_controller.py
Publish to ```/arm_des_pose```
With message type ```PointStamped```

## Requires
Servos plugged in and properly limited, but works with simulation


# Mecanum Controller
Given a desired linear and angular velocity for Maverick, compute the individual wheel speeds to achieve those, while minimizing the sum of the squares of the wheel velocities.

The robot is overactuated (4 independent actuators on 3 degrees of freedom: yaw, x-translation and y-translation). As such, you have four equations in three variables, for converting a desired Twist to wheel actions. We solve this with linear least squares.

Use this if you want to manually set some velocity for testing, or you are trying to use a joystick.

## Usage
    rosrun ieee2015_controller mecanum_controller.py
Publish to ```/twist```
With message type ```TwistStamped```

## Requires
To be useful, you should also start the xmega_connector node so that you can actually drive the robot

# Vehicle Controller
Given a desired position and current position, decide on a velocity based on some gain ```K``` multiplied by the square root of the position error.

This way, we accelerate as quickly as possible, and then spend the rest of the time in a state of constant negative acceleration, until the vehicle has stopped.

## Usage
    rosrun ieee2015_controller vehicle_controller.py

Publish desired pose to ```desired_pose``` and current pose to ```pose```
With messages of type ```PoseStamped```

## Requires
This node _will not work_ without knowlege of position. It won't do anything.


# TODO
- Make the desired velocity publish as TwistStamped (Haven't done this yet because it affects Gazebo, the other simulations, and the mecanum controller)