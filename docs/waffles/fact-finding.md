---  
title: Fact-Finding Missions!  
---

For the most part, everything that you have done with ROS in simulation so far is directly applicable to the real robots too. There are a few subtle differences though, and you should make sure that you are aware of these before going much further.

Complete the following *fact-finding missions*, which will help you to explore and identify the key differences between the way our TurtleBot3 Waffles work in the real world, compared to how they work in WSL-ROS (simulation).

#### Mission 1: Publishing Velocity Commands

In [Exercise 3 of Week 2](../../com2009/la1/week2/#ex3) you learnt how to publish velocity commands (from the command-line) to make the robot move in simulation. Return to that exercise now and repeat this, in simulation again (and come back here when you're done!)

Now, follow exactly the same steps, but this time on the robotics laptop, to make your real robot move instead.

What do you notice about how the real robot moves when you issue a correctly formulated `rostopic pub` command, e.g.:

```txt
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

(replacing some of the `0.0`s above with [applicable values](../../about/robots/#max_vels))

Next, look at the usage information for the `rostopic pub` command by entering:

***
```bash
rostopic pub -h
```
***

Try to work out how to control the publishing **rate** by adding an additional command-line argument to the end of the `rostopic pub` commands. Use this to specify a publishing rate of **1 hz** and see what happens (in terms of how the robot moves). Next, specify a rate of **10 hz** and see how this changes things.

When you stop the `rostopic pub` command what happens to the robot, and how does this differ to what happens when you do the same thing in simulation?

!!! question 
    Having answered the questions above, what implications might this have for any ROS nodes that you create to control the velocity of a *real* robot? 

#### Mission 2: The Camera Image Topic {#mission2}

In [Week 6](../../com2009/la1/week6) you worked extensively with the robot's camera and its images, which were published to the `/camera/rgb/image_raw` topic.

!!! warning
    The name of the camera image topic is not the same on the real robots!

On the laptop, use the ROS command-line tools that you know and love by now to interrogate the real robot ROS Network and identify the name of the camera image topic on the real robots.

#### Mission 3: Camera Image Resolution

At [the beginning of Week 6](../../com2009/la1/week6/#cam_img_questions) we looked at how to explore the messages published to the robot's camera image topic. Here you should have worked out which part of these messages told you about the *resolution* of the camera images (i.e.: the `height` and `width` of the images, in pixels). You may recall what this was, but if not, go back and interrogate this again in WSL-ROS to find out what resolution our robot's camera images are transmitted at, *in simulation* (you'll need to use `rostopic echo`). 

!!! warning 
    The real robot's camera captures images at a different image resolution! 

Use `rostopic echo` again, but this time on the Robotics Laptop, to interrogate the real robot ROS network and identify the `height` and `width` of the camera images that are captured by our real robot's camera.

!!! note 
    This will have implications when you come to apply image cropping techniques... the same cropping procedures may not work the same way for nodes run in simulation, compared to those running on a real robot!

#### Mission 4: Out of Range LiDAR Data

The robot's LiDAR sensor can only obtain measurements from objects within a certain distance range. In [Week 3](../../com2009/la1/week3/#range_max_min) we looked at how to work out what this range is, using the `rostopic echo` command. Apply the same techniques to the real robot now to discover the **maximum** and **minimum** distances that the real robot's LiDAR sensor can measure.

If the LiDAR sensor detects an object that falls within this range then it will report the exact distance to this object (in meters). Conversely, if it *doesn't* detect anything within this range then it will report a default *out-of-range* value instead. In simulation, the out-of-range value is `inf`.

!!! warning
    The *out-of-range* value reported by the real robot's LiDAR sensor is **not** `inf`!

Use the `rostopic echo` command to interrogate the ROS network running between your real Waffle and the robotics laptop, and find out what out-of-range value is used here.

#### Mission 5: Object Detection

In general, image detection gets a little more challenging in the real-world, where the same object might appear (to a robot's camera) to have slightly different colour tones under different light conditions, from different angles, in different levels of shade, etc. In [Exercise 3 of Week 6](../../com2009/la1/week6/#ex3) you may well have built an extremely robust `colour_search.py` node to detect each of the four coloured pillars in the `tuos_ros_simulations/coloured_pillars` world. See how well this now works in the real world now by running the same code on your real Waffle.

!!! question "Questions"
    1. Without changing any of your code, is the robot able to detect any of our *real* coloured pillars in the robot arena?
    1. Can you make any changes to adapt this and make it work more reliably in a real-world environment?


#### Summary

When working with the real robots we *strongly* recommend that you start off by developing your code in simulation, where it's a bit easier to test things out and less disastrous if things go wrong! Overall, you'll be able to develop things much faster this way. Whilst you're doing this though, keep in mind all the differences that you have identified during your fact finding missions above, so that there are less nasty surprises when you come to deploy your ROS applications on the real Waffles. 

Throughout the design phase, think about how your applications could be developed more flexibly to accommodate these variations, or how you could design things so that only small/quick changes/switches need to be made when you transition from testing in simulation, to deploying your applications in the real world. 