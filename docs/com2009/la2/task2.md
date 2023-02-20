---  
title: "Task 2: Obstacle Avoidance" 
---  

Develop the ROS node(s) that allow a TurtleBot3 Waffle robot to autonomously explore a simulated environment containing various obstacles. The robot must explore as much of the environment as possible in 90 seconds without crashing into anything!

**Assessment Format**: Simulation  
**Marks**: 15/100

## Summary

In [Lab Assignment #1, Week 3](../../la1/week3/#lidar) you were introduced to the LiDAR sensor on the robot, and learned what the data from this sensor tells us about the distance to any objects that are present in the robot's environment. In [Week 5 Exercise 4](../../la1/week5/#ex4) you should have used this data, in combination with the *ROS Action framework*, as the basis for a basic obstacle avoidance control system. Then, in [Advanced Exercise 1 (Week 5)](../../la1/week5/#adv_ex1) we discussed how this could be developed further into an effective *search strategy* by developing an action client node that would make successive calls to the action server to keep the robot moving randomly, and indefinitely, around an arena whilst avoiding obstacles.

This is one approach that you could use for this first task, but there are other (and potentially simpler) ways that this could be achieved too. 

Consider COM2009 Lecture 3 ("Sensing, Actuation & Control"), for instance, where you were introduced to *Cybernetic Control Principles* and some of *Braitenberg's "Vehicles"* were discussed and implemented on a Lego robot.  In particular, *"Vehicle 3b"* might well be relevant to consider as a simple method to achieve an obstacle avoidance behaviour.

Another aspect of this task is *exploration*: your robot will be awarded marks based on how much of the environment it is able to navigate around. Consider the search strategies that were discussed in Lecture 8 ("Local Guidance Strategies"), such as *"Brownian Motion"* and *"Levy walks,"* and how something along these lines could be implemented on the TurtleBot3 Waffle.

## Details

The simulated environment that your robot will need to explore for this will be a square arena of 5.0 m 5.0 m, so slightly bigger than the real robot arena in Diamond Computer Room 3. For the task, the arena will contain a number of *"obstacles,"* i.e.: short wooden walls and coloured cylinders. Your robot will need to be able to detect these obstacles and navigate around them in order to fully explore the space.

1. The robot will start in the centre of the arena (denoted "Zone 5").
1. It must explore the environment for 90 seconds without touching any of the arena walls or the obstacles within it.
1. If the robot makes contact with **anything** before the time has elapsed then the attempt will be stopped.
1. Nine equal-sized zones are marked out on the arena floor and the robot must enter as many of these as possible during the attempt.
1. The robot must be moving for the entire duration of the task. Simply just turning on the spot for the whole time doesn't count!

    <a name="launch"></a>

1. The ROS package that you submit must contain a launch file called `task2.launch`, such that the functionality that you develop for Task 2 can be launched from your package via the command:

    ```bash
    roslaunch com2009_team{} task2.launch
    ```

    **Test this out in WSL-ROS before submission to make sure that it works!**

1. This task will be assessed in simulation and the simulated environment will already be running before we attempt to execute your launch file. 

!!! note
    The location, orientation and quantity of obstacles in the arena will not be revealed beforehand, so the ROS package that you develop will need to be able to accommodate an unknown environment. 

## Simulation Resources

Within the `com2009_simulations` package there is an example arena which can be used to develop and test out your team's obstacle avoidance node(s) for this task. The simulation can be launched using the following `roslaunch` command:

```bash
roslaunch com2009_simulations obstacle_avoidance.launch
```

<a name="avoid_arena"></a>

<figure markdown>
  ![](figures/obstacle_avoidance.jpg)
  <figcaption>The `obstacle_avoidance` development arena.</figcaption>
</figure>

!!! tip
    The location, orientation and quantity of obstacles **will be different** for the final assessment of this task!

    Have a go at varying the environment yourselves, using the methods that we used in [Lab Assignment #1, Week 4](../../la1/week4/#sim-env-mods) to move, resize or delete existing objects, or add new simple box geometries.

## Marking

There are **15 marks** available for Task 2 in total, awarded based on the following criteria:

<center>

| Criteria | Marks | Details |
| :--- | :---: | :--- |
| **A**: Run time | 7/15 | You will be awarded marks for the amount of time that your robot spends exploring the environment before 90 seconds has elapsed, **or** the robot makes contact with anything in its environment ([as per the table below](#run-time)). |
| **B**: Exploration | 8/15 | You will be awarded 1 mark for each new arena zone that the robot manages to enter (excluding the one it starts in). The robot only needs to enter each zone once, but its full body must be inside the zone marking to be awarded the mark. |

</center>

Marks for "Run Time" will be awarded as follows:

<a name="run-time"></a>

<center>

| Time (Seconds) | Marks |
| :---: | :---: |
| 0-9 | 0 |
| 10-19 | 1 |
| 20-29 | 2 |
| 30-39 | 3 |
| 40-49 | 4 |
| 50-59 | 5 |
| 60-89 | 6 |
| The full 90! | 7 |

</center>