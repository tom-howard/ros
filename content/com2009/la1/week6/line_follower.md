+++  
title = "Week 6 Line Following"  
hidden = "true"
+++

#### The Code {#code}

Copy **all** the code below into your `line_follower.py` file.  Then, review the explainer [(below)](#explainer) to understand how this all works.

{{< include file="line_follower.py" code="true" lang="python" >}}

{{< nicenote warning "Fill in the Blank!" >}}
There is a method within the `Tb3Move()` class which allows us to publish a velocity command to the `/cmd_vel` topic. What is it? (Have a look at [the `tb3.py` source code](https://github.com/tom-howard/COM2009/blob/main/tuos_ros_examples/src/tb3.py) if you need a reminder).
{{< /nicenote >}}

#### The Code Explained {#explainer}

A lot of the things in here you will already be familiar with from the previous exercises, so we won't go into too much detail on all of this again.  The main thing you will notice is that we have once again built a class structure around this, which should now be familiar to you.  

In this case we want our robot to follow the line that is printed on the floor. We do this by applying the same image processing steps as in the previous exercises in [week 6](../week6/) to isolate the colours associated with the line and calculate its location in the robot's viewpoint.

The next step is to use the centroid component `cy` to determine how far the robot needs to turn in order to keep the line in the centre of its vision:

```python
y_error = cy - (width / 2)
        
kp = 1.0 / 50.0

fwd_vel = 0.1
ang_vel = kp * y_error

print(f"Y-error = {y_error:.3f} pixels, ang_vel = {ang_vel:.3f} rad/s")
self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
self.robot_controller.publish()
```

We are implementing *proportional control* here, which (if you are a COM2009/COM3009 student) you should know about from Prof Moore's lecture on PID Control (Lecture 6).  Ideally, we want the centre of the line on the floor to be in the centre of the robot's viewpoint at all times: this is our *target position*.  The *actual position* is where the line on the floor actually is, i.e.: the `cy` centroid component.  The *position error* is then the difference between the *actual* and *target* position:

```python
y_error = cy - (width / 2)
```

The only way we can reduce this error is by changing the robot's angular velocity.  The robot always needs to travel with forward velocity, so we define a fixed value of 0.1 m/s at all times to achieve this:

```python
fwd_vel = 0.1
```

In order to correct for our *position error*, we multiply it by a proportional gain (`kp`), which will provide us with an angular velocity that *should* start to make the error reduce:

```python
ang_vel = kp * y_error
```

We then simply set these two velocities in our `robot_controller` object and then publish them to the `/cmd_vel` topic using methods from the `Tb3Move()` class:

```python
self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
self.robot_controller.{BLANK}    # publish the velocity command
```

We're doing all this inside the camera callback function, so that this process will repeat every time a new image is obtained.  If the proportional gain is set appropriately, this should ensure that our *position error* (`y_error`) is always kept to a minimum, so that the robot follows the line!
