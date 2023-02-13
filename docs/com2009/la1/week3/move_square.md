---  
title: "Week 3 'move_square' Python Template"  
---

**A combined publisher-subscriber node to achieve odometry-based control...**

Below you will find a template Python script to show you how you can both publish to `/cmd_vel` and subscribe to `/odom` in the same node.  This will help you build a *closed-loop* controller to make your robot follow a **square motion path** of size: **1m x 1m**. 

You can publish velocity commands to `/cmd_vel` to make the robot move, monitor the robot's *position* and *orientation* in real-time, determine when the desired movement has been completed, and then update the velocity commands accordingly.  

## Suggested Approach

Moving in a square can be achieved by switching between two different movement states sequentially: *Moving forwards* and *turning* on the spot. At the start of each movement step we can read the robot's current odometry, and then use this as a reference to compare to, and to tell us when the robot's position/orientation has changed by the required amount, e.g.:

1. With the robot stationary, **read the odometry** to determine its current X and Y position in the environment.
1. **Move forwards** until the robot's X and Y position indicate that it has moved linearly by 0.5m.
1. **Stop** moving forwards.
1. **Read the robot's odometry** to determine its current orientation ("yaw"/<code>&theta;<sub>z</sub></code>).
1. **Turn on the spot** until the robot's orientation changes by 90&deg;.
1. **Stop** turning.
1. Repeat.  

```python title="move_square.py"
--8<-- "code/move_square.py"
```

## Alternative Approach: Waypoint Tracking

A square motion path can be fully defined by the co-ordinates of its four corners, and we can make the robot move to each of these corners one-by-one, using its odometry system to monitor its real-time position, and adapting linear and angular velocities accordingly.

This is slightly more complicated, and you might want to wait until you have a bit more experience with ROS before tackling it this way (we'll also cover this in the COM2009 lecture course).

<p align="center">
  <a href="../../week3#ex1_ret">&#8592; Back to Week 3 - Exercise 1</a>
</p>