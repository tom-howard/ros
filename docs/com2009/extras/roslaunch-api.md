---  
title: Using the ROSlaunch Python API  
---  

# Using the ROSlaunch Python API

The *ROSlaunch API* allows you to launch nodes (and other launch files) from inside your own nodes! [The official documentation for the roslaunch API](http://wiki.ros.org/roslaunch/API%20Usage) is the best place to go to find out how to use this, but here's a quick example, if you're short on time. 

Recall [the SLAM Exercise from Week 3](../../la1/week3/#ex3). Here we had SLAM running in the background, while we drove our robot around a simulated environment (using the `turtlebot3_teleop` node). Once we'd generated a full map we used a `rosrun` command (in the terminal) to save a copy of this map: 

```bash
rosrun map_server map_saver -f {map name}
```

Suppose we wanted to do this programmatically instead...  

The following is an example of a very simple ROS node that does just that. Once the code below is executed it will launch the `map_saver` node from the `map_server` package in exactly the same way as above (but using the ROSlaunch API instead). The key difference here is that instead of `{map name}`, we need to define the full file path for the map that we want to generate.

```python title="map_saver.py"
--8<-- "code/map_saver.py"
```

You could wrap this into one of your own ROS nodes, or even build a standalone node inside your package to call the `map_saver` node when required. Perhaps you could even wrap the last three of the above commands inside a while loop and get the node to update a map file at regular intervals (say 5 seconds?) while your robot explores its environment...
