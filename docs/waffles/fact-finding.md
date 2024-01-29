---  
title: Fact-Finding Missions!  
---

For the most part, everything that you do with ROS in simulation is directly applicable to the real Waffles too. There are a few subtle differences though, and you should make sure that you are aware of these whilst working on Assignment #2.

Complete the following *fact-finding missions*, which will help you to explore and identify the key differences between the way our TurtleBot3 Waffles work in the real world, compared to how they work in simulation.

Each mission is linked to a particular part of the Assignment #1 course and, ideally, you should have completed the Assignment #1 tasks before completing the corresponding fact finding mission below. 

Be mindful of the differences that we are trying to highlight here and the implications that they will have on the applications that you develop for Assignment #2. 

### Mission 1: Publishing Velocity Commands

!!! success "Assignment #1 Checkpoint"
    You'll need to have completed [Part 2 Exercise 3](../com2009/assignment1/part2.md#ex3) before starting on this mission.

In the above Assignment #1 exercise you learnt how to publish velocity commands from the command-line to make the robot move in simulation. Repeat this in simulation if you need a reminder on how it all worked.

Then, follow exactly the same steps but with the real Waffle this time... What do you notice about how the real robot moves when you issue a correctly formulated `rostopic pub` command, e.g.:

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

(replacing some `0.0`s above with [applicable values](../about/robots.md#max_vels))

Next, look at the usage information for the `rostopic pub` command by entering:

```bash
rostopic pub -h
```

Try to work out how to control the publishing **rate** by adding a command-line argument to the end of the `rostopic pub` commands. Use this to specify a publishing rate of **1 hz** and see what happens (in terms of how the robot moves). Next, specify a rate of **10 hz** and see how this changes things.

When you stop the `rostopic pub` command what happens to the robot, and how does this differ to what happens when you do the same thing in simulation?

!!! question 
    Having answered the questions above, what implications might this have for any ROS nodes that you create to control the velocity of a *real* robot? 

### Mission 2: The Camera Image Topic {#mission2}

!!! success "Assignment #1 Checkpoint"
    You'll need to have completed the whole of [Part 6](../com2009/assignment1/part6.md) before starting on this mission.

In Part 6 you worked extensively with the robot's camera and its images, which were published to the `/camera/rgb/image_raw` topic.

!!! warning
    **The name of the camera image topic is not the same on the real robots!**

On the laptop, use ROS command-line tools such as `rostopic list` and `rostopic info` to interrogate the real robot ROS Network and identify the name of the alternative camera image topic that is used here.

### Mission 3: Camera Image Resolution

!!! success "Assignment #1 Checkpoint"
    You'll need to have completed the questions before [Part 6 Exercise 1](../com2009/assignment1/part6.md#cam_img_questions) before starting on this mission.

At the start of Part 6 we explore the messages published to the robot's camera image topic. Here you need to work out which part of these messages indicate the *resolution* of the camera images (i.e.: the `height` and `width` of the images, in pixels). You may recall what this was, but if not, go back and interrogate this again to find out what resolution the *simulated* robot's camera images are transmitted at (you'll need to use `rostopic echo`). 

!!! warning 
    **The real robot's camera captures images at a different image resolution!** 

On the Robotics Laptop use `rostopic echo` again, to interrogate the real robot ROS network and identify the `height` and `width` of the camera images that are captured by our real robot's camera.

!!! note 
    This will have implications when you come to apply image cropping techniques... the same cropping procedures may not work the same way for nodes run in simulation, compared to those running on a real robot!

### Mission 4: Out of Range LiDAR Data

!!! success "Assignment #1 Checkpoint"
    You'll need to have completed the section on ["Interpreting `/LaserScan` Data" (Part 3)](../com2009/assignment1/part3.md#interpreting-laserscan-data) before starting on this mission.

The robot's LiDAR sensor can only obtain measurements from objects within a certain distance range. In Part 3 we look at how to work out what this range is, using the `rostopic echo` command. Apply the same techniques to the real robot now to discover the **maximum** and **minimum** distances that the real robot's LiDAR sensor can measure.

If the LiDAR sensor detects an object that falls within this range then it will report the exact distance to this object (in meters). Conversely, if it *doesn't* detect anything within this range then it will report a default *out-of-range* value instead. In simulation, the out-of-range value is `inf`.

!!! warning
    The *out-of-range* value reported by the real robot's LiDAR sensor is **not** `inf`!

Use the `rostopic echo` command to interrogate the ROS network running between your real Waffle and the robotics laptop, and find out what out-of-range value is used here.

### Mission 5: Object Detection

!!! success "Assignment #1 Checkpoint"
    You'll need to have completed [Part 6 Exercise 3](../com2009/assignment1/part6.md#ex3) before starting on this mission.

In general, image detection gets a little more challenging in the real-world, where the same object might appear (to a robot's camera) to have slightly different colour tones under different light conditions, from different angles, in different levels of shade, etc. In simulation, you may build an extremely effective `colour_search.py` node to detect each of the four coloured pillars in the `tuos_simulations/coloured_pillars` world. See how well this now works in the real world now by running the same code on your real Waffle.

!!! question "Questions"
    1. Without changing any of your code, is the robot able to detect any of our *real* coloured pillars in the robot arena?
    2. Can you make any changes to adapt this and make it work more reliably in a real-world environment?

### Summary

When working on Assignment #2 you will naturally do a fair bit of the development work in simulation, where it's easier to test things out and less disastrous if things go wrong! Overall, you'll be able to develop things much faster this way, and you can do this outside of your weekly lab sessions too. Whilst you're doing this though, keep in mind all the differences that you have identified during the above fact-finding missions, so that there are less nasty surprises when you come to deploy your ROS applications on the real Waffles. 

Throughout the design phase, think about how your applications could be developed more flexibly to accommodate these variations, or how you could design things so that only small/quick changes/switches need to be made when you transition from testing in simulation, to deploying in the real world. 