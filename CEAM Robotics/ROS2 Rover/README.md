# ROS2 Differential Drive Rover

A ROS2-based differential-drive rover simulation developed as part of my robotics work through CEAM.

The project focuses on understanding the connection between **ROS2 velocity commands, differential-drive kinematics, robot modelling, Gazebo simulation, and PID control logic**.

## Overview

The rover is modelled as a two-wheel differential-drive robot.

The system accepts velocity commands through the ROS2 `/cmd_vel` topic and calculates the corresponding left and right wheel speeds.

The project includes:

- A ROS2 Python control node
- Differential-drive wheel-speed calculations
- Basic PID control logic for both wheels
- A URDF robot description
- A Gazebo/SDF rover model
- ROS2 launch files
- A simulated environment

## System Architecture

```text
              ROS2
                │
                │ /cmd_vel
                ▼
       ┌──────────────────┐
       │ Diff Drive       │
       │ Controller       │
       └────────┬─────────┘
                │
        ┌───────┴────────┐
        ▼                ▼
   Left Wheel        Right Wheel
   Velocity           Velocity
        │                │
        ▼                ▼
     PID Logic        PID Logic
```

The Gazebo model separately provides the simulated rover and differential-drive interface.

## Differential Drive Kinematics

The commanded linear velocity `v` and angular velocity `ω` are converted into wheel angular velocities.

For wheel separation `L` and wheel radius `r`:

$$
\omega_L =
\frac{v-\omega L/2}{r}
$$

$$
\omega_R =
\frac{v+\omega L/2}{r}
$$

The rover uses:

- Wheel radius: `0.05 m`
- Wheel separation: `0.30 m`

## PID Control

A basic PID controller is implemented in Python for the left and right wheels.

The controller uses:

- `Kp = 1.0`
- `Ki = 0.01`
- `Kd = 0.1`

The PID calculation combines proportional, integral, and derivative terms to calculate a control output from the wheel-speed error.

## Robot Model

The rover contains:

- A rectangular chassis
- Left and right wheels
- Continuous/revolute wheel joints
- Collision and visual geometry
- Inertial properties

The main chassis dimensions are:

```text
Length: 0.50 m
Width:  0.30 m
Height: 0.10 m
```

The wheel radius is `0.05 m`.

## ROS2 Node

The custom `diff_drive_controller` node:

- Subscribes to `/cmd_vel`
- Reads linear and angular velocity commands
- Calculates left and right wheel speeds
- Calculates wheel-speed errors
- Runs PID calculations
- Logs the resulting values

## Project Structure

```text
ROS2 Rover/
│
├── rover_control/
│   ├── __init__.py
│   └── diff_drive_controller.py
│
├── urdf/
│   └── rover.urdf
│
├── models/
│   └── rover/
│       ├── model.sdf
│       └── model.config
│
├── launch/
│   ├── rover.launch.py
│   └── gazebo.launch.py
│
├── worlds/
│   └── empty.world
│
├── resource/
│   └── rover_control
│
├── package.xml
├── setup.py
└── setup.cfg
```

## Technologies

- ROS2
- Python
- rclpy
- geometry_msgs
- URDF
- SDF
- Gazebo / Ignition Gazebo
- Differential-drive kinematics
- PID control

## What I Learned

Through this project, I explored:

- ROS2 package structure
- ROS2 nodes and topics
- Publishing/subscribing concepts
- Differential-drive kinematics
- Robot modelling with URDF and SDF
- Gazebo simulation
- Basic PID control
- Connecting high-level velocity commands with robot motion

## Current Limitations

The current PID implementation uses placeholder actual wheel velocities rather than feedback from wheel encoders or a simulated joint-state source.

The project therefore represents an early-stage control implementation rather than a complete closed-loop wheel controller.

## Future Improvements

Planned improvements include:

- Adding actual wheel-state feedback
- Connecting PID outputs to simulated wheel control
- Adding encoder-based velocity estimation
- Integrating the controller more tightly with Gazebo
- Adding odometry
- Adding sensors and localization
- Integrating path tracking algorithms such as Pure Pursuit
- Moving toward a fully autonomous ROS2 rover

## Author

**Surya**

Second-year Robotics and AI student interested in autonomous systems, robotics, embedded systems, and control.