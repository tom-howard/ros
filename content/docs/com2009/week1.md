# Week 1

## Introduction

In this first week you will learn the basics of ROS and become familiar with some of the key tools and principles of this framework, which will allow you to program robots and work with ROS applications effectively.  For the most part, you will interact with ROS using the *Linux command line* and so you will also become familiar with some key Linux command line tools that will help you.  Finally, you will learn how to create some basic ROS Nodes using Python.

### Intended Learning Outcomes

By the end of this session you will be able to:
1. Control a TurtleBot3 Robot, in simulation, using ROS.
1. Launch ROS applications using `roslaunch` and `rosrun`.
1. Interrogate running ROS applications using key ROS command line tools.
1. Create a ROS package comprised of multiple nodes and program these nodes (in Python) to communicate with one another using ROS Communication Methods.
1. Navigate a Linux filesystem and learn how to do various filesystem operations from within a Linux Terminal.

## Packages

ROS applications are organised into *packages*.  ROS packages are collections of scripts, parameters and configurations relating to some common robot functionality, and ROS uses packages as a way to organise all the programs running on a robot.  **The package system is a fundamental concept in ROS and all ROS programs are organised in this way**. 

1. If you haven't done so already, launch your WSL-ROS environment by running the WSL-ROS shortcut in the Windows Start Menu.  This should open up a *terminal application* and an *Ubuntu terminal instance*.  We'll refer to this terminal instance as **TERMINAL 1**.
1. In the terminal enter the following command to launch a simulation of a TurtleBot3 Waffle in an empty world:  
        
    ***
    **TERMINAL 1:**
        
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ***

    > **Note:** *A `$` symbol indicates a command that should be entered directly into a Linux Terminal. Enter (or copy and paste) all the text after the `$` into the terminal **excluding the `$`!*** 
 
1. A Gazebo simulation window should open and within this you should see a TurtleBot3 Waffle (similar to the real robots that you will be working with later in this course):

![](/figures/gz_tb3_empty_world.png)

    