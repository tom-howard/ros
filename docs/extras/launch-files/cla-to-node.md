---  
title: Passing Command-line Arguments to Python Nodes via ROSlaunch  
---  

# Passing Command-line Arguments to Python Nodes via ROSlaunch

Having built a CLI for our Python Node on the previous page, we'll now look at how this all works when we call a node using `roslaunch` instead. 

1. First, create a launch file called `publisher_cli.launch`. It probably makes sense to create this one inside your `week1_pubsub` package:

    ```bash
    roscd week1_pubsub/launch
    ```
    ```bash
    touch publisher_cli.launch
    ```
    
1. Inside this add the following:

    ```xml
    <launch>
      <arg name="roslaunch_colour" default="Black" />
      <arg name="roslaunch_number" default="0.999" />

      <node pkg="tuos_ros_examples" type="publisher_cli.py"
        name="publisher_cli_node" output="screen"
        args="-colour $(arg roslaunch_colour) -number $(arg roslaunch_number)" />
    </launch>
    ```

    Here, we are specifying command-line arguments for *the launch file* using `<arg>` tags, [as discussed earlier](../roslaunch-clas):

    ```xml
    <arg name="roslaunch_colour" default="Black" />
    <arg name="roslaunch_number" default="0.999" />    
    ```

    Next, we use a `<node>` tag to launch the `publisher_cli.py` node from the `tuos_ros_examples` package. All of that should be familiar to you from [Week 1](../../la1/week1/#package_attributes). What's new here however is the additional `args` attribute:

    ```xml
    args="-colour $(arg roslaunch_colour) -number $(arg roslaunch_number)"
    ```

    This is how we pass the `roslaunch` command-line arguments into the CLI of our *ROS Node*:

    * `-colour` and `-number` are the command-line arguments of the **Python Node**
    * `roslaunch_colour` and `roslaunch_number` are the command-line arguments of the **launch file**

1. Run the launch file without passing any arguments first, and see what happens:

    ```bash
    roslaunch week1_pubsub publisher_cli.launch
    ```

1. Then try again, this time setting alternative values for the available command-line arguments:

    ```bash
    roslaunch week1_pubsub publisher_cli.launch roslaunch_colour:=purple roslaunch_number:=4
    ```