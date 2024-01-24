---  
title: Passing Command-line Arguments to Python Nodes via ROSlaunch  
---  

!!! success "COM2009 Assignment #1 Checkpoint"
    It helps if you've already completed [Assignment #1 **Part 1**](../../com2009/assignment1/part1.md) before working on this.

Having built a CLI for our Python Node on the previous page, we'll now look at how this all works when we call a node using `roslaunch` instead. 

1. First, create a launch file called `publisher_cli.launch`. It probably makes sense to create this inside your `part1_pubsub` package:

    ```bash
    roscd part1_pubsub/launch
    ```
    ```bash
    touch publisher_cli.launch
    ```
    
1. Inside this add the following:

    ```xml
    <launch>
      <arg name="roslaunch_colour" default="Black" />
      <arg name="roslaunch_number" default="0.999" />

      <node pkg="tuos_examples" type="publisher_cli.py"
        name="publisher_cli_node" output="screen"
        args="-colour $(arg roslaunch_colour) -number $(arg roslaunch_number)" />
    </launch>
    ```

    Here, we are specifying command-line arguments for *the launch file* using `<arg>` tags, [as discussed earlier](./roslaunch-clas.md):

    ```xml
    <arg name="roslaunch_colour" default="Black" />
    <arg name="roslaunch_number" default="0.999" />    
    ```

    Next, we use a `<node>` tag to launch the `publisher_cli.py` node from the `tuos_examples` package. All of that should be familiar to you from [Part 1](../../com2009/assignment1/part1.md#package_attributes). What's new here however is the additional `args` attribute:

    ```xml
    args="-colour $(arg roslaunch_colour) -number $(arg roslaunch_number)"
    ```

    This is how we pass the `roslaunch` command-line arguments into the CLI of our *ROS Node*:

    * `-colour` and `-number` are the command-line arguments of the **Python Node**
    * `roslaunch_colour` and `roslaunch_number` are the command-line arguments of the **launch file**

1. Run the launch file without passing any arguments first, and see what happens:

    ```bash
    roslaunch part1_pubsub publisher_cli.launch
    ```

1. Then try again, this time setting alternative values for the available command-line arguments:

    ```bash
    roslaunch part1_pubsub publisher_cli.launch roslaunch_colour:=purple roslaunch_number:=4
    ```