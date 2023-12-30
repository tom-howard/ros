---
title: "Part 6 Line Following"  
---

# Part 6 Line Following

Copy **all** the code below into your `line_follower.py` file.  Then, review the annotations to understand how it all works.

```py title="line_follower.py"
--8<-- "snippets/line_follower.py"
```

1. A lot of the things in here you will already be familiar with from the previous exercises, so we won't go into too much detail on all of this again.  The main thing you will notice is that we have once again built a class structure around this, which should now be familiar to you from previous weeks of this course.  

2. In this case we want our robot to follow the line that is printed on the floor. We do this by applying the same image processing steps as in the previous exercises, to isolate the colours associated with the line and calculate its location in the robot's viewpoint.

3. We'll use the centroid component `cy` to determine how far the robot needs to turn in order to keep the line in the centre of its vision:

4. We are implementing *proportional control* here.

    ??? tip "COM2009 Students!"
        PID control was covered by Prof Moore in Lecture 6!
    
    Ideally, we want the centre of the line on the floor to be in the centre of the robot's viewpoint at all times: this is our *target position*.  The *actual position* is where the line on the floor actually is, i.e.: the `cy` centroid component.  The *position error* is then the difference between the *actual* and *target* position:

5. The only way we can reduce this error is by changing the robot's angular velocity.  The robot always needs to travel with forward velocity, so we define a fixed value at all times to achieve this. 

6. In order to correct for our *position error*, we multiply it by a proportional gain (`kp`), which will provide us with an angular velocity that *should* start to make the error reduce.
    
    If the proportional gain is set appropriately, this should ensure that our *position error* (`y_error`) is always kept to a minimum, so that the robot follows the line!

7. We then simply set these two velocities in our `robot_controller` object and then publish them to the `/cmd_vel` topic using methods from the `Tb3Move()` class. 

    !!! warning "Fill in the Blank!"
        There is a method within the `Tb3Move()` class which allows us to publish a velocity command to the `/cmd_vel` topic. What is it? (Have a look at [the `tb3.py` source code](https://github.com/tom-howard/tuos_ros/blob/main/tuos_examples/src/tb3.py) if you need a reminder).

<p align="center">
  <a href="../../part6#ex4_ret">&#8592; Back to Part 6 - Exercise 4</a>
</p>