---  
title: "A Move Square Python Template for Part 2"  
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
--8<-- "snippets/move_square.py"
```

1. Import the `Twist` message for publishing velocity commands to `/cmd_vel`.
2. Import the `Odometry` message, for use when subscribing to the `/odom` topic.
3. Import the `euler_from_quaternion` function to convert orientation from quaternions (as provided in the `Odometry` message) to Euler angles (about [the principal axes](../../part2/#principal-axes)).
4. Finally, import some useful mathematical operations (and `pi`), which you may find useful:

    | Mathematical Operation | Python Implementation |
    | :---: | :---: |
    | $\sqrt{a+b}$ | `#!python sqrt(a+b)` |
    | $a^{2}+(bc)^{3}$ | `#!python pow(a, 2) + pow(b*c, 3)` |
    | $\pi r^2$ | `#!python pi * pow(r, 2)` |

5. Obtain the relevant data from the `/odom` topic. For this, we need to know the robot's position and orientation in its environment, all of which is contained within the "pose" part of the `Odometry` message:

    ```python
    pose = topic_data.pose.pose
    ```

    From there, we can separate out the position and orientation parts:

    ```python
    position = pose.position
    ```
    
    ```python
    orientation = pose.orientation
    ``` 

6. Here we obtain the robot's **current** position coordinates.

7. And here we obtain the robot's current orientation (in quaternions) and convert it to Euler angles (in radians) about [the principal axes](../../part2/#principal-axes), where:
    * "roll" = <code>&theta;<sub>x</sub></code>
    * "pitch" = <code>&theta;<sub>y</sub></code>
    * "yaw" = <code>&theta;<sub>z</sub></code>

8. We're only interested in `x`, `y` and <code>&theta;<sub>z</sub></code>, so we assign these to class variables `self.x`, `self.y` and `self.theta_z`, so that we can access them elsewhere within our `Square()` class.

9. Sometimes, it can take a few moments for the first topic message to come through, and it's useful to know when that's happened so that you know you are dealing with actual topic data! Here, we're just setting a flag to `True` once the callback function has executed for the first time (i.e. the first topic message *has* been received).

10. This might be useful in the `#!python main()` class method (below), to switch between turning and moving forwards...

11. This node needs to **read** message data from one topic (`/odom`) and **write** messages on another (`/cmd_vel`), so we need to set up a subscriber and a publisher here accordingly. 

12. Here, we define some variables that we can use to store relevant bits of odometry data while our node is running (and read it back to implement feedback control):
    * `self.x`, `self.y` and `self.theta_z` will be used by the callback function to store the robot's **current** pose
    * `self.x0`, `self.y0` and `self.theta_z0` can be used in the `main()` method to keep a record of where the robot **was** at a given moment in time (and determine how far it has moved since that point)

13. Here we establish a `Twist` message, which we can populate with velocities and then publish to `/cmd_vel` within the `main()` method in order to make the robot move.

14. Publish an empty `Twist` message to stop the robot (by default all velocities will be zero).

## Alternative Approach: Waypoint Tracking

A square motion path can be fully defined by the coordinates of its four corners, and we can make the robot move to each of these corners one-by-one, using its odometry system to monitor its real-time position, and adapting linear and angular velocities accordingly.

This is slightly more complicated, and you might want to wait until you have a bit more experience with ROS before tackling it this way (we'll also cover this in the COM2009 lecture course).

<p align="center">
  <a href="../../part2#ex5_ret">&#8592; Back to Part 2 - Exercise 5</a>
</p>