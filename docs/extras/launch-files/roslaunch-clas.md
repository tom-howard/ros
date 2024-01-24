---  
title: ROSlaunch Command-line Arguments  
--- 

!!! success "COM2009 Assignment #1 Checkpoint"
    It helps if you've already completed [Assignment #1 **Part 1**](../../com2009/assignment1/part1.md) before working on this.


A lot of the launch files that we have use throughout COM2009 Assignment #1 actually have *Command-Line Arguments* (CLAs) that can be (optionally) supplied when making the `roslaunch` call. Consider the `turtlebot3_empty_world.launch` file for instance...

Enter the full command and then press the ++space++ and ++tab++ keys on your keyboard like so:

    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch[SPACE][TAB][TAB]

This will reveal the available *command-line arguments* for this launch file:

    model  x_pos  y_pos  z_pos

We can therefore *optionally* specify custom values for these parameters in order to change what happens when the launch file is executed. Try this, for example:

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch x_pos:=1 y_pos:=1
```

When the Gazebo simulation is launched, the robot will be located at coordinates `(1, 1)` in the `X-Y` plane, rather than `(0, 0)`, as would usually be the case.

!!! note
    We assign values to `roslaunch` command-line arguments using the `:=` operator. 

This is all made possible through the use of `<arg>` tags at the start of a launch file (just after the opening `<launch>` tag):

```xml
<launch>
  <arg name="x_pos" default="0.0"/>
  <arg name="y_pos" default="0.0"/>
  ...
```

Within these tags we need to define two attributes:

1. `name`: the **name** we want to give to the CLA (`name="x_pos"`, `name="y_pos"` etc...)
1. `default`: A default value in order to make the CLA *optional*

    (In the above example, the robot will be located at coordinates `(0, 0)` *by default*, unless we specify otherwise.)