---  
title: Using the ROSlaunch Python API  
---  

!!! success "COM2009 Assignment #1 Checkpoint"
    It helps if you've already completed Assignment #1 [up to and including all of **Part 3**](../../com2009/assignment1/part3.md) before working on this.

The *ROSlaunch API* allows us to launch nodes (and other launch files) from inside our own nodes! [The official documentation for the roslaunch API](http://wiki.ros.org/roslaunch/API%20Usage) is the best place to go to find out how to use this, but here's a quick example, if you're short on time. 

In [the SLAM Exercise in Assignment #1 Part 3](../../com2009/assignment1/part3.md#ex2) we run SLAM in the background while driving our Waffle around a simulated environment manually, using the `turtlebot3_teleop` node. Once SLAM has generated a full map we use a `rosrun` command (in the terminal) to call a ready-made node bundled into ROS, which saves a copy of this map for us: 

```bash
rosrun map_server map_saver -f {map name}
```

Suppose we wanted to do this programmatically instead...  

The following is an example of a very simple ROS node that does just that. Once the code below is executed it will launch the `map_saver` node from the `map_server` package in exactly the same way as above (but using the ROSlaunch API instead). The key difference here is that instead of `{map name}`, we need to define the full file path for the map that we want to generate.

```python title="map_saver.py"
--8<-- "snippets/map_saver.py"
```

You could wrap this into one of your own ROS nodes, or even build a standalone node inside your package to call the `map_saver` node when required. Perhaps you could even wrap the last three of the above commands inside a while loop and get the node to update a map file at regular intervals (say 5 seconds?) while your robot explores its environment...

??? hint
    You may need to wrap this in a loop in order for it to work reliably.
