---  
title: Building Command-Line Interfaces (CLIs) for Python Nodes  
---  

!!! success "COM2009 Assignment #1 Checkpoint"
    It helps if you've already completed [Assignment #1 **Part 1**](../../com2009/assignment1/part1.md) before working on this.

Suppose we have a very simple node such as [the `publisher.py` node that you create in Part 1](../../com2009/assignment1/part1/publisher.md). This node publishes `std_msgs/String` type messages to a topic called `"chatter"`. Let's have a look at an alternative version of that node that takes in command-line arguments and publishes *those* to the `"chatter"` topic instead.

Take a look at [the `publisher_cli.py` node from the `tuos_examples` package](https://github.com/tom-howard/tuos_ros/blob/main/tuos_examples/src/publisher_cli.py). 

Here, we use [the Python `argparse` module](https://realpython.com/command-line-interfaces-python-argparse/) to allow us to work with arguments that are passed to the node from the command-line:

```python
import argparse
```

We instantiate `argparse` in the `__init__()` method of the `Publisher()` class to build a *command-line interface* (CLI) for the node:

```python
cli = argparse.ArgumentParser(...)
cli.add_argument(...)
```
(See [the code](https://github.com/tom-howard/tuos_ros/blob/main/tuos_examples/src/publisher_cli.py) for more details)

Arguments that we define with a `-` at the front will be *optional*, i.e. we don't have to provide these every time we run the node. We *do*, however, need to assign a *default value* for each optional argument in cases where no value is supplied, e.g.:

```python
cli.add_argument(
    "-colour",
    metavar="COL",
    default="Blue", 
    ...
```

The final step is to grab any arguments that *are* passed to this node when it is called. We use the `rospy.myargv()` method here, so that this works regardless of whether we call the node using `rosrun` or `roslaunch`: 

```python
self.args = cli.parse_args(rospy.myargv()[1:])
```

Having defined the CLI above, `argparse` then automatically generates *help text* for us! Try running the following command to see this in action:

```bash
rosrun tuos_examples publisher_cli.py -h
```

!!! warning
    You need to have a *ROS Master* running in order for this to work, so do this in another terminal instance by running the `roscore` command.

Run the node as it is (using `rosrun`) and see what happens:

```bash
rosrun tuos_examples publisher_cli.py
```

Stop the node (using ++ctrl+c++) and then run it again, but this time providing a value for the `number` variable, via the CLI:

```bash
rosrun tuos_examples publisher_cli.py -number 1.5
```

??? info
    We can assign values to *Python* command-line arguments using a space (as above) or the `=` operator (`-number=1.5`). 

Stop the node again (using ++ctrl+c++) and also stop the ROS Master that you enabled in another terminal instance.