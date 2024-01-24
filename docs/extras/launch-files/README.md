---  
title: Launch Files  
---  

## Overview

As you should know from [the COM2009 ROS Course](../../com2009/README.md), *Nodes* can be executed on a ROS network in two ways:

1. Using the `rosrun` command:

    ``` { .bash .no-copy }
    rosrun {package name} {script name}
    ```
    
 2. Or by using `roslaunch`:

    ``` { .bash .no-copy }
    roslaunch {package name} {launch file}
    ```

In [Assignment #1 Part 1](../../com2009/assignment1/part1.md#launch-files) you learn that a *launch file* is an `xml` file with a `.launch` file extension. Inside this we can ask ROS to do a number of different things from one single `roslaunch` command-line call.

`roslaunch` offers a number of advantages over `rosrun`:

* **Multiple nodes** can be executed **simultaneously**.
* `roslaunch` will launch the **ROS Master** for us, if isn't already running (so we don't have to manually call `roscore`).
* From within one `.launch` file, we can launch other `.launch` files.
* We can pass in **additional arguments** to launch certain things *conditionally*, or pass specific arguments to certain ROS nodes.
* As if that wasn't enough, there is also a [`roslaunch` Python API](http://wiki.ros.org/roslaunch/API%20Usage), allowing us to launch nodes from within other (Python) nodes!

In this section we'll talk about some of these advanced features that you may find useful.