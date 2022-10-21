+++  
title = "Week 3 'move_square' Python Template"  
hidden = "true"
+++

## A combined publisher-subscriber node to achieve odometry-based control

Below you will find a template Python script to show you how you can both publish to `/cmd_vel` and subscribe to `/odom` in the same node.  This will help you build a *closed-loop* controller to make your robot follow a **square motion path** of size: **0.5m x 0.5m**. 

You can publish velocity commands to `/cmd_vel` to make the robot move, monitor the robot's *position* and *orientation* in real-time, determine when the desired movement has been completed, and then update the velocity commands accordingly.  

{{< include file="move_square.py" code="true" lang="python" >}}