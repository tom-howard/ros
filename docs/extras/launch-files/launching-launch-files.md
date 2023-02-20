---  
title: Launching Other Launch Files  
---  

# Launching Other Launch Files

Think back to [the `move_circle.py` node that you created in Week 2](../../la1/week2/#ex4). We need a simulation of our robot active in order to run this, which we can enable with the following command (which you should be very familiar with by now):

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Suppose you wanted to save yourself some work and launch both the simulation *and* the `move_circle.py` node at the same time...

1. Firstly, return to your `week2_navigation` package and create a `launch` directory (if you haven't done so previously):

    ```bash
    roscd week2_navigation
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
      <node pkg="week2_navigation" type="move_circle.py" name="circle_node" output="screen" />
    </launch>
    ```

    The `<node>` tag should already be familiar to you, but the `<include>` tag before it might not be...

    This is what we use to launch other launch files. Here, we are locating the `turtlebot3_gazebo` package (using `find`), and asking for the `turtlebot3_empty_world.launch` file to be executed.

1. From the command-line, execute your newly created launch file as follows:

    ```bash
    roslaunch week2_navigation circle.launch
    ```

    The TurtleBot3 Waffle will be launched in the *"empty world"* simulation, and the robot will start moving in a circle straight away!