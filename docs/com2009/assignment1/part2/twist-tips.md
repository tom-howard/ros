---  
title: Working with Twist Messages in Python
---

# Working with Twist Messages in Python

From [the Part 1 publisher exercise](../../part1/#ex5), we know how to publish a `String` type message to a topic in Python, but how do we apply the same principles to a `Twist` message (on the `/cmd_vel` topic)? Let's have a look at this... 

First, you need to import the `rospy` library, as well as the `Twist` message type from the `geometry_msgs` library:

```python
import rospy
from geometry_msgs.msg import Twist
```

Then, create an instance of a `rospy.Publisher()` and assign it to an object called `pub`. When we create the object we tell the `Publisher()` method which topic we want to publish this message to (via the first input argument), and also that we will be publishing a message of the `Twist` type (the second input argument):

```python
pub = rospy.Publisher({topic name}, Twist, queue_size=10) # a queue size of 10 usually works!
```

Then we need to create a `Twist()` message instance and assign it to an object (which we'll call `vel_cmd`):

```python
vel_cmd = Twist()
```

We know from earlier that the `geometry_msgs/Twist` message has the format:

    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z

We also know, that only velocity commands issued to the following two parameters will actually have any effect on the velocity of our robot:

    geometry_msgs/Vector3 linear
      float64 x

...and:

    geometry_msgs/Vector3 angular
      float64 z

As such, we set appropriate velocity values to these attributes of the `Twist()` message (assigned to `vel_cmd`):

```python
vel_cmd.linear.x = 0.0 # m/s
vel_cmd.angular.z = 0.0 # rad/s
```

We can then publish this to the relevant topic on the ROS network by supplying it to the `rospy.Publisher().publish()` method (which we instantiated as `pub` earlier):

```python
pub.publish(vel_cmd)
```

Use these pointers when working on your `move_circle.py` node!

<p align="center">
  <a href="../../part2#ex4_ret">&#8592; Back to Part 2 - Exercise 4</a>
</p>