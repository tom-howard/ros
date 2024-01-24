---  
title: Launching Other Launch Files  
---  

!!! success "COM2009 Assignment #1 Checkpoint"
    It helps if you've already completed [**Part 2** up to and including **Exercise 4**](../../com2009/assignment1/part2.md) before working on this as, in this example, we'll build on [the `move_circle.py` node](../../com2009/assignment1/part2.md#ex4).

We need a simulation of our Waffle active in order to run this, which we can enable with the following command:

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Suppose you wanted to save yourself some work and launch both the simulation *and* the `move_circle.py` node at the same time...

1. Firstly, navigate to your `part2_navigation` package and create a `launch` directory (if you haven't done so):

    ```bash
    roscd part2_navigation
    ```
    ```bash
    mkdir launch
    ```
    ```bash
    cd launch/
    ```

1. Then create a *launch file*. You could call this whatever you want, but for the purposes of this example we'll create one called `circle.launch`:

    ```bash
    touch circle.launch
    ```

1. Inside this file add the following:

    ```xml
    <launch>
      <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_empty_world.launch" />
      <node pkg="part2_navigation" type="move_circle.py" name="circle_node" output="screen" />
    </launch>
    ```

    Having completed [Assignment #1 Part 1](../../com2009/assignment1/part1.md), the `<node>` tag should already be familiar to you. The `<include>` tag before it however, might not be...

    This is what we use to launch other launch files. Here, we are locating the `turtlebot3_gazebo` package (using `find`), and asking for the `turtlebot3_empty_world.launch` file to be executed.

1. From the command-line, execute your newly created launch file as follows:

    ```bash
    roslaunch part2_navigation circle.launch
    ```

    The Waffle will be launched in the *"empty world"* Gazebo environment, and the robot should start moving in a circle straight away!