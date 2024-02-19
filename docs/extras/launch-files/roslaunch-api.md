---  
title: Using the ROS Launch Python API  
---  

# Using the ROS Launch Python API 

!!! success "COM2009 Assignment #1 Checkpoint"
    It helps if you've already completed Assignment #1 [up to and including all of **Part 3**](../../com2009/assignment1/part3.md) before working on this.

The *ROS Launch Python API* allows us to launch nodes (and other launch files) from inside our own nodes! [The official documentation for the ROS Launch API](http://wiki.ros.org/roslaunch/API%20Usage) is the best place to go to find out how to use this, but here's a quick example, if you're short on time. 

Within the `tuos_examples` package there is a node called `pointless_file_generator`.

!!! tip
    You may need to [update your local copy of the course repo](../tuos-ros.md#updating) to ensure that you have this!

This node takes a file path as a command-line argument, and writes a text file of the same name to the file system. From the command line, you can call the node like this:

```bash
rosrun tuos_examples pointless_file_generator -f FILENAME
```

A text file called `FILENAME.txt` will be saved to your current working directory (use the `#!sh pwd` to determine your current working directory, and the `#!sh ls` command to show the files/directories present at this file system location).

Suppose you wanted to call this node programmatically instead (i.e. from within another Python Node). Below is an example of a very simple ROS node that does just that. Once the code below is executed it will launch the `pointless_file_generator` node from the `tuos_examples` package, but this time using the ROS Launch API. The key difference here is that instead of `FILENAME`, we need to define a **full path** to the text file that we want to generate.

```python title="roslaunch_api_example.py"
--8<-- "snippets/roslaunch_api_example.py"
```

Create a node in one of your own ROS packages and then copy and paste the above code into it. You'll need to update the `#!py file_path = "/full/path/to/text/file"` line to represent the path to a **real** location on your file system. As long as the location already exists (i.e. the folder structure) then the file will be created automatically by the node in the specified location. You must specify a **full file system path** in order for this to work correctly though (use the `pwd` command).

## Assignment #2 Hint: Saving a SLAM Map Programmatically 

In [the SLAM Exercise in Assignment #1 Part 3](../../com2009/assignment1/part3.md#ex2) we run SLAM in the background while driving our Waffle around a simulated environment manually, using the `turtlebot3_teleop` node. Once SLAM has generated a full map we use a `rosrun` command (in the terminal) to call a `map_saver` node to save a copy of this map for us: 

```bash
rosrun map_server map_saver -f {map name}
```

You can therefore use *exactly the same approach* as in the example above to achieve this *programmatically*!

Wrap this into one of your own Task 4 ROS nodes, or build a standalone node inside your team's ROS package to call the `map_saver` node when required. Perhaps you could even wrap the relevant lines of code from the above example inside a while loop and get the node to update a map file at regular intervals (say every 5 seconds?) while your robot explores its environment...

!!! warning

    1. When calling the `map_saver` node you may actually find that it **must** be wrapped within a loop in order for the process to work reliably.
    1. **Don't** call the `map_saver` node too regularly though (i.e. no more than once every few seconds **at most**), otherwise you'll swamp the ROS network which can result in the system becoming unstable and the robot becoming unresponsive. 
