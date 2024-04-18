---
title: "Assignment #2: Team Robotics Project"
--- 

## Overview

In Assignment #2 you will put into practice everything that you learn about ROS in Assignment #1, and explore the capabilities of the framework further.

You will attend a 2-hour lab session per week in Diamond Computer Room 5 for the full 12-week semester. You will work **in teams** to develop ROS Nodes for our TurtleBot3 Waffles that allow them to successfully complete a series of robotics tasks, most of which in a *real-world environment*. There are **four tasks** to complete in total, all of which will be assessed as part of Assignment #2.

## The Tasks

There are **four tasks** in total that you must complete for this Assignment. Each task is marked and goes towards your final mark for Assignment #2. Tasks ^^1, 2 & 4^^ will ~~all~~ be assessed on a real TurtleBot3 Waffle in the robot arena in Computer Room 5, ^^Task 3 will be assessed in simulation^^. All will be marked based on how well the robot completes each of the tasks. An overview of the tasks and the marking breakdown is shown in the table below. 

<center>

| Task | Details | Assessment Format | Marks |
| :---: | :---   | :---: | :---: |
| 1 | [Velocity Control](./parta/task1.md) | Real World | 10/100 |
| 2 | [Avoiding Obstacles](./parta/task2.md) | Real World | 20/100 |
|   | [An *'out-of-the-box' submission* for Part A](./parta/submission.md) | - | 10/100 |
| 3 | [Maze Navigation](./partb/task3.md) | Simulation | 20/100 |
| 4 | [Exploration & Search](./partb/task4.md) | Real World | 40/100 |

</center>

As shown above, there are **100 marks** available in total for Assignment #2.

## Assessment

This assignment is worth 30% of the overall mark for the COM2009 course. As a team you will be assessed on a ROS package that you develop to satisfy the above tasks.

**For Tasks 1, 2 & 4**:  
Each submission will be assessed by extracting your ROS package on one of the robotics laptops that you will use extensively in the labs throughout the assignment. Nodes within your package will then be executed on the laptop to control a real robot in the Diamond Computer Room 5 Robot Arena.

**For Task 3**:  
Each submission will be extracted into [the WSL-ROS environment](../../software/wsl-ros/README.md) running on one of the WSL-ROS laptops (also available during the lab sessions). Nodes within your package will then be executed to control a TurtleBot3 Waffle in a *simulated* environment.

**Feedback**:  
Within 3 weeks of submission you will receive your marks and a recording of the assessment so that you can see how well your robot performed in each of the tasks, and so that you can see exactly how your marks were awarded!

### Submission Deadlines

The assignment is essentially split into two separate parts (A & B), with two submission deadlines as summarised in the table below (see Blackboard for exact dates and times).

<center>

| Part | Task(s) | Deadline |  
| :---: | :---: | :---: |
| A | 1 & 2 | Week 6 |
| B | 3 & 4 | Week 12 | 

</center>

For each submission, you'll need to upload a ROS package (as a `.tar` file) to a submission portal on Blackboard. 

Before you get started on any of the programming tasks you should (as a team) create a single ROS package (further details on the next page). You can then add all the necessary functionality for each task as you go along. For each submission, you'll then need to create a copy of your package in its current state by creating a `.tar` archive of it, and submit this to Blackboard by the specified deadline ([the export process is explained here](submission.md)). 

!!! note
    You should work on each task **as a team**, and you only need to make one submission per team for each task.

## Your ROS Package

### Launching Your Code

In order to launch the necessary functionality within your package for a given task you will need to include correctly named *launch files*: `task1.launch`, `task2.launch`, etc. This will allow you to ensure that all the required functionality is executed when your submission is assessed, and also ensures that we know exactly how to launch this functionality in order to assess it. Full details of the requirements for each launch file are provided on the associated task page.

!!! warning 
    It's up to **you** to ensure that your code launches as intended for a given task. If it doesn't, then you'll be awarded zero marks, so **make sure you test it all out prior to submission**!

For more information on how to create `.launch` files, refer to the following resources:

1. [Assignment #1, Part 1, Exercise 8: Creating a Launch File](../assignment1/part1.md#ex8)
2. The [ROS Extras: **Launch Files**](../../extras/launch-files/README.md) section of this course site 

### Key Requirements

In order to be awarded any marks for any task outlined in the table above, you **must** ensure that the following key requirements are met in regard to the ROS package that you submit (as well as any additional requirements specific to a given task):

1. Your package must be submitted to Blackboard as a `.tar` file with the following naming convention:

        com2009_team{}.tar
  
    Where the `{}` is replaced with your own team number. [See here for how to create a `.tar` archive of your package](submission.md).
  
1. Your ROS package directory, when extracted, must be named:

        com2009_team{}/

    Again, replacing the `{}` with your own team number!

1. Your ROS **package name** must also be the same, so that the following would work (for example):

    ```bash
    roslaunch com2009_team100 task1.launch
    ```

    (assuming you are Team 100!)

1. Finally (and most importantly), your ROS package must work *"out-of-the-box,"* i.e. the Teaching Team won't make any modifications or fix any errors for you! 

!!! warning
    Failure to follow these requirements could result in you being awarded **zero marks**!