+++
title = "Week 2: Odometry & Basic Navigation"
weight = 2
description = "In this session you'll learn how to control a ROS robot's velocity (and thus its position), how to interpret Odometry data and implement some open-loop control nodes."
+++

{{% textalign center %}}
*You should be able to complete the exercises on this page within a two-hour lab session*.
{{% /textalign %}}

{{% textalign left %}}
[<i class="fas fa-solid fa-arrow-left"></i> Previous: "Week 1: ROS & Linux Basics"](../week1)
{{% /textalign %}}

## Introduction

### Aims

This week you will learn how to control a ROS robot's *position* and *velocity* from both the command line and through ROS Nodes. You will also learn how to interpret data that allows us to monitor a robot's position in its physical environment.  The things you will learn here form the basis for all robot navigation in ROS, from simple open-loop methods to more advanced closed-loop control (which you will learn more about next week).

### Intended Learning Outcomes



### Quick Links

* [Exercise 1: Exploring Odometry Data](#ex1)
* [Exercise 2: Creating a Python node to process Odometry data](#ex2)
* [Exercise 3: Moving a Robot with `rostopic` in the Terminal](#ex3)
* [Exercise 4: Creating a Python node to make the robot move](#ex4)