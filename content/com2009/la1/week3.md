+++  
title = "Week 3: Advanced Navigation & SLAM"  
weight = 3  
description = "Implement odometry-based velocity control to make a robot follow a pre-defined motion path. Explore the LiDAR sensor, how the data form this device can be of huge benefit for robotics applications, and see this in practice by leveraging the autonomous navigation (and mapping) tools within ROS."  
+++

{{% textalign center %}}
*You should be able to complete the exercises on this page within a two-hour lab session*.
{{% /textalign %}}

{{% textalign left %}}
[<i class="fas fa-solid fa-arrow-left"></i> Previous: "Week 2: Odometry & Basic Navigation"](../week2)
{{% /textalign %}}

## Introduction

### Aims

This week you will implement *closed-loop velocity control* and create a ROS node that can control a robot's motion path by using odometry data as a feedback signal. In doing this however, you will start to appreciate the limitations of odometry as a feedback signal, which will lead us on to exploring some other data-streams that could be used to aid navigation further. Finally, you will leverage some existing ROS libraries and TurtleBot3 packages to explore some of the *autonomous navigation* methods that are available within ROS.

### Intended Learning Outcomes

By the end of this session you will be able to:
1. Combine both publisher *&* subscriber communication methods (that you have so far dealt with in isolation) into a single Python node to implement closed-loop (odometry-based) velocity control of a robot.
1. Explain the limitations of Odometry-based motion control methods. 
1. Interpret the data that is published to the `/scan` topic and use existing ROS tools to visualise this.
1. Use existing ROS tools to implement SLAM and build a map of an environment. 
1. Leverage existing ROS libraries to make a robot navigate an environment *autonomously*, using the map that you have generated.
1. Explain how these SLAM and Navigation tools are implemented and what information is required in order to make them work.

### Quick Links

* [Exercise 1: Make your robot follow a Square motion path](#ex1)

### Additional Resources

* 

## Getting Started

1. Launch your WSL-ROS environment by running the WSL-ROS shortcut in the Windows Start Menu (if you haven't already done so). Once installed, the *Windows Terminal* app should launch with an *Ubuntu terminal instance* ready to go (**TERMINAL 1**).
1. Also launch VS Code now by [following the steps here to launch it correctly within the WSL-ROS environment](/wsl-ros/vscode).

### Restoring your Environment

Remember that any work that you do in WSL-ROS will not be preserved between sessions or across different University computers. You should have run the `wsl_ros` tool at the end of the previous session to back up your home directory to your University U: Drive. Restore this now before you go any further by running the following command in **TERMINAL 1**:

***
**TERMINAL 1:**
```bash
wsl_ros restore
```
***

### Launching the Robot Simulation

You should know exactly how to do this now but, just to re-iterate, enter the following into **TERMINAL 1**:
        
***
**TERMINAL 1:**
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
***

...which will launch a Gazebo simulation of a TurtleBot3 Waffle in an empty world:

![](/images/gazebo/tb3_empty_world.png?width=800px)

{{% nicenote tip %}}
Getting bored with entering that long command to launch the simulation? You could use the `tb3_empty_world` alias instead!
{{% /nicenote %}}

## Odometry-based Navigation

In the previous session you created [a Python node to make your robot move using *open-loop control*](../week2/#ex4). To achieve this you published velocity commands to the `/cmd_vel` topic to make the robot follow a circular motion path.

{{% nicenote note "Questions" %}}
* How do you know if your robot actually achieved the motion path that you were hoping for?
* In a real-world environment, what external factors might result in your robot *not* achieving its desired trajectory?
{{% /nicenote %}}

Last week you also learnt about [Robot Odometry](../week2/#odometry), which is used by the robot to keep track of its *position* and *orientation* in the environment.  This is determined by a process called *"dead-reckoning"*, which is only really an approximation, but it's a fairly good one in any case, and we can use this as a feedback signal to understand if our robot is moving in the way that we expect it to.  We can therefore build on the techniques that we used in the `move_circle.py` node from last time, and now also build in the ability to *subscribe* to a topic too. In this case, we'll be subscribing to the `/odom` topic that we worked with a bit (in isolation) last time, and use this to provide us with a feedback signal to allow us to implement some basic *closed-loop control*.

#### Exercise 1: Make your robot follow a Square motion path {#ex1}

1. Launch a new terminal instance (**TERMINAL 2**) and, from there, navigate to the `week2_navigation` package that you created last time.
    {{< nicenote tip "Hint" >}}
You can use the `roscd` command for this!
    {{< /nicenote >}}
1. Navigate to the package `src` directory and use the Linux `touch` command to create a new file called `move_square.py`:
    
    ***
    **TERMINAL 2:**
    ```bash
    touch move_square.py
    ```
    ***

1. Then make this file executable using `chmod`:

    ***
    **TERMINAL 2:**
    ```bash
    chmod +x move_square.py
    ```
    ***

1. Use the VS Code File Explorer to navigate to this `move_square.py` file and open it up, ready for editing.
1. [There's a template here to help you with this exercise](move_square). Copy and paste the template code into your new `move_square.py` file to get you started.
1. Run the code as it is to see what happens...

    {{< nicenote warning "Fill in the Blank!" >}}
Something not quite working as expected? We may have missed out [something very crucial](../week1/subscriber/#dfts) on **the very first line** of the code template, can you work out what it is?!
    {{< /nicenote >}}

1. Fill in the blank as required and then adapt the code to make your robot follow a **square motion path** of **0.5m x 0.5m** dimensions:
    * The robot's odometry will tell you how much the robot has moved and/or rotated, and so you should use this information to achieve the desired motion path. 
    * Your Python node will therefore need to *subscribe* to the `/odom` topic as well as *publish* to `/cmd_vel`.

##### Advanced features:

1. Adapt the node further to make the robot automatically stop once it has performed two complete loops.
1. Create a launch file to launch this *and* the `odom_subscriber.py` node from last time simultaneously!

After following a square motion path a few times, your robot *should* return to the same location that it started from.




{{% textalign right %}}
[Next: "Week 4: ROS Services" <i class="fas fa-solid fa-arrow-right"></i>](../week4)
{{% /textalign %}}