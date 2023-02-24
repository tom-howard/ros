---  
title: Overview  
---  

## The Tasks

There are **five tasks** in total that you must complete for Lab Assignment #2. Each task is marked and goes towards your final grade for Lab Assignment #2. Tasks will be assessed either in simulation or on a real robot. The assessment format for each task and the overall marking breakdown is shown in the table below. There are **100 marks** available, in total, for Lab Assignment #2.

<center>

| Task | Details | Marks | Assessment Format |
| :---: | :---   | :---: | :--- |
| 1 | [A deployable ROS Package](../task1) | 20/100 | Real Robots |
| 2 | [Obstacle Avoidance](../task2) | 15/100 | Simulation |
| 3 | [Maze Navigation](../task3) | 15/100 | Simulation |
| 4 | [Detection, Search & Beaconing](../task4) | 15/100 | Simulation |
| 5 | [Real-World Exploration](../task5) | 35/100 | Real Robots | 

</center>

## Submission Details

There are two submission deadlines, summarised in the table below (see Blackboard for exact dates and times).

<center>

| Submission | Task(s) | Deadline | 
| :---: | :---: | :---: |
| A | Task 1 | Week 9 |
| B | Tasks 2, 3, 4 & 5 | Week 12 | 

</center>

For each submission, you'll need to provide a ROS package (as a `.tar` file) to a submission portal on Blackboard. 

Before you get started on any of the programming tasks you should (as a team) create a single ROS package (further details on the next page). You can then add all the necessary functionality for each task as you go along. For each submission, you'll then need to create a copy of your package in its current state by creating a `.tar` archive of it, and submit this to Blackboard by the specified deadline ([the export process is explained here](../submission)). 

!!! note
    You should work on each task **as a team**, and you only need to make one submission per team for each task.

## Assessment

**For simulation-based tasks**  
Your team's submission will be assessed by extracting and running your package in the same WSL-ROS environment that you have been working with throughout Lab Assignment #1.

**For real-robot-based tasks**  
Your team's submission will be assessed by the Teaching Team by extracting and running your package on one of our robotics laptops, that you will use extensively throughout Weeks 7-12. Your submission will be used to control a real robot in the Diamond Computer Room 3 Robot Arena.

Regardless of the assessment type, you will receive a recording of the assessment afterwards so that you can see how well your robot performed in each of the tasks, and so that you can see exactly how your marks were awarded!

## Launching Your Code

In order to launch the necessary functionality within your package for a given task you will need to include correctly named *launch files*, `task1.launch`, `task2.launch`, etc. This will allow you to ensure that all the required functionality is executed when your submission is assessed, and also ensures that we know exactly how to launch this functionality in order to assess it. Full details of the requirements for each launch file are provided on the associated task page.

!!! warning 
    It's up to **you** to ensure that your code launches as intended for a given task. If it doesn't, then you'll be awarded zero marks, so **make sure you test it all out prior to submission**! 

## Key Requirements

In order to be awarded any marks for any task outlined in the table above, you **must** ensure that the following key requirements are met in regard to the ROS package that you submit (as well as any additional requirements specific to a given task):

1. Your package must be submitted to Blackboard as a `.tar` file with the following naming convention:

        com2009_team{}.tar
  
    Where the `{}` is replaced with your own team number. [See here for how to create a `.tar` archive of your package](../submission).
  
1. Your ROS package directory, when extracted, must follow a similar naming convention:

        com2009_team{}/

    Again, replacing the `{}` with your own team number!

1. Your ROS **package name** must also be the same, so that the following would work (for example):

    ```bash
    roslaunch com2009_team100 task1.launch
    ```

    (assuming you are Team 100!)

1. Finally (and most importantly), your ROS package must work *"out-of-the-box"*, i.e. the Teaching Team won't be able to make any modifications or fix any errors for you! 

!!! warning
    Failure to follow these requirements could result in you being awarded **zero marks**!