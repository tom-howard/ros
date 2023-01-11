---  
title: "Lab 2: Feedback Control"
weight: 2  
---  

## Introduction

In Lab 1 we explored how ROS works and how to bring a robot to life. Let's quickly recap the key points:

**ROS Nodes**

* Are executable programs that perform specific robot tasks and operations.
* Typically, there'll be many ROS Nodes running on a robot simultaneously in order to make it work.
* We can create our own Nodes on top of what's already running, to add extra functionality.
* You may recall that we created our own ROS Node in Python, to make our TurtleBot3 Waffle follow a square motion path.

**Topics and Messages**

* All the ROS Nodes running on a network can communicate and pass data between one another using a Publisher/Subscriber based Communication Principle.
* ROS Topics are key to this - they are essentially the communication channels (or the plumbing) on which all data is passed around between the nodes.
* Different topics communicate different types of information.
* Any Node can publish (*write*) and/or subscribe to (*read*) any ROS Topic in order to pass information around or make things happen.
* One of the key ROS Topics that we worked with last time was `/cmd_vel`, which is a topic that communicates velocity commands to make a robot move.
* We published `Twist` messages to this (both via the command line, and in Python) to make our TurtleBot3 Waffle move.

**Open-Loop Control**

We used a time-based method to control the motion of our robot in order to get it to generate a square motion path. This type of control is *open-loop*: we hoped that the robot had moved (or turned) by the about that was required, but had no *feedback* to tell us whether this had actually been achieved.

In this lab we'll look at how this can be improved, making use of some of our robot's on-board sensors to tell us where the robot is or what it can see in its environment, in order to achieve a goal more reliably and be able to better adapt to changes and uncertainty in the environment.

### Aims

In this lab, we'll build some ROS Nodes (in Python) that incorporate data from some of our robot's sensors. This sensor data is published to specific topics on the ROS Network, and we can build ROS Nodes to *subscribe* to these. We'll see how the data from these sensors can be used as *feedback* to inform decision-making, thus allowing us to implement some different forms of *closed-loop control*, making our robot more autonomous.

### Intended Learning Outcomes

By the end of this session you will be able to:

1. 

### Quick Links

* [Exercise 1: ](#ex1)
* [Exercise 2: ](#ex2)
* [Exercise 3: ](#ex3)
* [Exercise 4: ](#ex4)

## The Lab

### Getting Started

#### Downloading the AMR31001 ROS Package

To start with, you'll need to download a ROS package to the Robot Laptop that you are working on today. This package contains all the resources that you'll need for the lab exercises.

1. Open up a terminal instance on the laptop, either by using the `Ctrl+Alt+T` keyboard shortcut, or by clicking the Terminal App icon in the favourites bar on the left-hand side of the desktop:
    
    ![](/images/laptops/bash_terminal_icon.svg?width=60px)
        
    (we'll refer to this as **TERMINAL 1**).

1. In **TERMINAL 1**, run the following commands in order:

    ***
    **TERMINAL 1:**

    1. `wget -O build.sh ###`

    1. `chmod +x build.sh`

    1. `./build.sh`

    ***

#### Launching ROS

Much the same as last time, you'll now need to get ROS up and running on your robot. 

1. First, identify the number of the robot that you have been provided with. Robots are named as follows:

    ```txt
    dia-waffleNUM
    ```
    ... where `NUM` is a unique *'Robot Number'* (a number between 1 and 50).

1. In **TERMINAL 1** type the following command to *pair* the laptop and robot:

    ***
    
    **TERMINAL 1:**
    ```bash
    waffle NUM pair
    ```
    Replacing `NUM` with the number of the robot that you have been provided with.
    
    ***

1. Enter the password for the robot when requested (we'll tell you what this is in the lab).

    You *may* see a message like this early on in the pairing process:

    ![](/images/laptops/ssh_auth.svg?width=14cm)

    If so, just type `yes` and then hit `Enter` to confirm that you want to continue.

1. Once the pairing process is finished you should see a message saying `pairing complete`, displayed in blue in the terminal. 

1. Then, in the same terminal (**TERMINAL 1**), enter the following command:

    ***
    **TERMINAL 1:**
    ```bash
    waffle NUM term
    ```
    (again, replacing `NUM` with the number of *your* robot).
    
    ***

    Any text that was in the terminal should now disappear, and a green banner should appear across the bottom of the terminal window:
    
    ![](/images/laptops/tmux.svg?width=14cm)

    Remember, this is a terminal instance running *on the robot*, and any commands that you enter here will be *executed on the robot* (not the laptop!)

1. Now, launch ROS on the robot by entering the following command:

    ***
    **TERMINAL 1:**
    ```bash
    roslaunch tuos_tb3_tools ros.launch
    ```
    ***

    After a short while, you should see a message like this:

    ```txt
    [INFO] [#####] Calibration End  
    ```

    ROS is now up and running, and you're ready to go!

    You can close down this terminal instance now.

### Odometry System (explainer)

Discuss all the sensors on board the robot that can be used to inform control...



#### Exercise 1: Odometry-based Turning

Demonstrate how to use odometry to control the turn angle, then ask the students to do the same for forward velocity...


### The LiDAR Sensor

#### Exercise 2: Wall detection




#### Exercise 3: SLAM and Navigation




### The Camera

#### Exercise 4: Object Tracking with PID??

(maybe a simulation-based exercise, to make life easier?)