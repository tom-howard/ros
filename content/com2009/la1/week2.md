+++
title = "Week 2: Odometry & Basic Navigation"
weight = 2
description = "In this session you'll learn how to control a ROS robot's velocity (and thus its position), how to interpret Odometry data and implement some open-loop control nodes."
+++

{{% textalign center %}}
*You should be able to complete the exercises on this page within a two-hour lab session*.
{{% /textalign %}}

{{% textalign left %}}
[<i class="fas fa-solid fa-arrow-left"></i> Previous: "Week 1: ROS & Linux Basics"](../week1)
{{% /textalign %}}

## Introduction

### Aims

This week you will learn how to control a ROS robot's *position* and *velocity* from both the command line and through ROS Nodes. You will also learn how to interpret data that allows us to monitor a robot's position in its physical environment.  The things you will learn here form the basis for all robot navigation in ROS, from simple open-loop methods to more advanced closed-loop control (which you will learn more about next week).

### Intended Learning Outcomes

By the end of this session you will be able to:
1. Interpret the Odometry data published by a ROS Robot and identify the parts of these messages that are relevant to a 2-wheeled differential drive robot (such as the TurtleBot3).
1. Develop Python nodes to obtain Odometry messages from an active ROS network and *translate* them to provide useful information about a robot's *pose* in a convenient, human-readable way.
1. Implement *open-loop velocity control* of a robot using ROS command-line tools.
1. Develop Python nodes that use open-loop velocity control methods to make a robot follow a pre-defined motion path.

### Quick Links

* [Exercise 1: Exploring Odometry Data](#ex1)
* [Exercise 2: Creating a Python node to process Odometry data](#ex2)
* [Exercise 3: Moving a Robot with `rostopic` in the Terminal](#ex3)
* [Exercise 4: Creating a Python node to make the robot move](#ex4)

### Additional Resources

* [ROS Odometry Messages Explained]()
* [A `Twist` Message Usage Example]()

## Getting Started

1. If you haven't done so already, launch your WSL-ROS environment by running the WSL-ROS shortcut in the Windows Start Menu. As you will now know, this may take a couple of minutes, but once it's ready this will open up the Windows Terminal and an *Ubuntu terminal instance* (which we'll refer to as **TERMINAL 1**).

1. It's also worth launching VS Code now, so that it's ready to go for when you need it later on. [Follow the steps here to launch it correctly](/wsl-ros/vscode/).

### Restoring your Environment

Remember that any work that you do within this WSL-ROS Environment will not be preserved between sessions or across different University computers.  [At the end of the previous session](../week1/#backup) you should have run the `wsl_ros` tool to back up your home directory to your University U: Drive. Restore this now before you start on this Week 2 session by running the following command in **TERMINAL 1**:

***
**TERMINAL 1:**
```bash
wsl_ros restore
```
***

### Downloading some ROS Packages for this Course

GIT CLONE COM2009


### Launching the Robot Simulation

In the terminal enter the following command to launch a simulation of a TurtleBot3 Waffle in an empty world:  
        
***
**TERMINAL 1:**
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
***

A Gazebo simulation window should open and within this you should see a TurtleBot3 Waffle in empty space:

![](/images/gazebo/tb3_empty_world.png?width=800px)

## Position and Velocity

Two types of *Velocity Command* can be issued to any ROS Robot to make it move (and thus change its *position*):

* **Linear** Velocity: The velocity at which the robot moves *forwards* or *backwards* in one of its principal axes.
* **Angular** Velocity: The velocity at which the robot *rotates* about one of its principal axes.

### Principal Axes {#principal-axes}

The motion (i.e. the velocity) of any mobile robot can be defined in terms of *three* principal axes: `X`, `Y` and `Z`. In the context of our TurtleBot3 Waffle, these axes (and the motion about them) are defined as follows:

![](/figures/tb3_axes.svg?width=20cm)

In theory then, a robot can move linearly or angularly about *any* of these three axes, as shown by the arrows in the figure. That's six *Degrees of Freedom* (DOFs) in total, achieved based on a robot's design and the actuators it is equipped with. 

You should hopefully recall from the ["Introducing the Robots" page](/about/robots/#tb3) that our TurtleBot3 Waffles only have two motors though, so they don't actually have six DOFs! These two motors can be controlled independently, which provides a *"differential drive"* configuration, and which ultimately provides it with a total of **two degrees of freedom** in total, as illustrated below.

![](/figures/tb3_velocity.svg?width=20cm)

It can therefore only move **linearly** in the **x-axis** (*Forwards/Backwards*) and **angularly** in the **z-axis** (*Yaw*).

It's also worth noting that our TurtleBot3 Waffle robots have **maximum velocity limits**, which were also defined in the ["Introducing the Robots" page](/about/robots/#tb3).

{{% nicenote note "Question" %}}
What are the maximum velocity limits of our robots?
{{% /nicenote %}}

### ROS Velocity Commands

In the previous session you learnt how to [list all the topics that are currently active on a ROS system](../week1/#rostopic). Open up a new terminal instance now (**TERMINAL 2**) and use what you learnt previously to *list* all the topics that are active on the ROS network now, as a result of launching the Gazebo simulation earlier.

{{% nicenote note "Question" %}}
1. Which topic in the list do you think could be used to control the velocity of the robot?
2. Use the `rostopic info` command on the topic to find out more about it.
{{% /nicenote %}}

The topic you identified[^cmd_vel] should use a message of the `geometry_msgs/Twist` type. You'll have to send messages of this type to this topic in order to make the robot move. Use the `rosmsg` command ([as you did in Exercise 4 last week](../week1/#ex4)) to find out more about the format of this message[^rosmsg_info].

[^cmd_vel]: The topic is `/cmd_vel` (i.e. *command velocity*).
[^rosmsg_info]: Answer: `rosmsg info geometry_msgs/Twist`.

You should now be looking at a message format that looks like this: 

```txt
geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
```

There are six parameters that we can assign values to here: `linear.x`, `linear.y`, `linear.z`; and `angular.x`, `angular.y`, `angular.z`. These are the robot's six degrees of freedom, relating to its three principal axes, as we discussed above. These topic messages are therefore formatted to give a ROS Programmer the ability to *ask* a robot to move in any one of its six DOFs. 

```txt
geometry_msgs/Vector3 linear
  float64 x  <-- Forwards (or Backwards)
  float64 y  <-- Left (or Right)
  float64 z  <-- Up (or Down)
geometry_msgs/Vector3 angular
  float64 x  <-- "Roll"
  float64 y  <-- "Pitch"
  float64 z  <-- "Yaw"
```

As we also learnt above though, our TurtleBots can only actually move with **linear** velocity in the **x**-axis and **angular** velocity in the **z**-axis. As a result, only velocity commands issued to the `linear.x` (Forwards/Backwards) or `angular.z` ("Yaw") parts of this message will have any effect.

## Robot Odometry

#### Exercise 1: Exploring Odometry Data {#ex1}

Another topic that should have appeared when you ran `rostopic list` above is `/odom`. This topic contains *Odometry data*, which is also essential for robot navigation and is a basic feedback signal, allowing a robot to approximate its location.

1. In **TERMINAL 2** use the `rostopic echo` command to display the odometry data currently being published by our simulated robot:

    ***
    **TERMINAL 2:**
    ```bash
    rostopic echo -c /odom
    ```
    ***

    Expand the terminal window as necessary so that you can see the whole topic message (it starts with `header` and ends with `---`).
    
    {{< nicenote note "Question" >}}
What does the `-c` option in the command above actually do?
    {{< /nicenote >}}

1. Now, you need to launch a new Windows Terminal instance so that you can view it side-by-side with **TERMINAL 2**. To do this, press the "New Tab" button whilst pressing the `Shift` key. We'll call this one **TERMINAL 3**. Arrange both windows side-by-side, so you can see what's happening in both, simultaneously.

1. In **TERMINAL 3** launch the `turtlebot3_teleop_keyboard` node [as you did last week](../week1/#ex1):

    ***
    **TERMINAL 3:**
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
    ***

1. In **TERMINAL 3** enter `A` a couple of times to make the robot rotate on the spot.  Observe how the odometry data changes in **TERMINAL 2**.  Is there anything in the `twist` part of the `/odom` message that corresponds to the `angular vel` that you are setting in **TERMINAL 3**? 
1. Now press the `S` key to halt the robot, then press `W` a couple of times to make the robot drive forwards.  How does the `twist` part of the message now correspond to the `linear vel` setting in **TERMINAL 3**?
1. Now press `D` a couple of times and your robot should start to move in a circle.  What linear and angular velocities are you requesting in **TERMINAL 3**, and how are these represented in the `twist` part of the `/odom` message?  What about the `pose` part of the message?  How is this data changing as your robot moves in a circular path.
    
    {{< nicenote note "Question" >}}
What do you think `twist` and `pose` are actually telling us?
    {{< /nicenote >}}

1. Press `S` in **TERMINAL 3** to halt the robot (but leave the `turtlebot3_teleop_keyboard` node running).  Then, press `Ctrl+C` in **TERMINAL 2** to shut down the `rostopic echo` process. 

1. Next, with the robot stationary, use `rosrun` to run a Python node that we have created to help illustrate what odometry data actually represents in terms of the robot's potion and orientation in its environment: 

        [TERMINAL 2] $ rosrun com2009_examples robot_odometry.py
        
1. Now (using the keyboard teleop node in **TERMINAL 3**), drive your robot around again, keeping an eye on the outputs that are being printed by the `robot_odometry.py` node in **TERMINAL 2** as you do so.

    The output of the `robot_odometry.py` node shows you how the robot's odometry is changing in real-time as you move the robot around. The `"initial"` column tells us the robot's odometry (its position and orientation) when the node was first launched, and the `"current"` column show us what it currently is. The `"delta"` column then simply shows the difference between the two.  ***Which odometry parameters haven't changed, and is this what you would expect (considering the robot's principal axes [as illustrated above](#fig_principal_axes))?***

1. Press `Ctrl+C` in **TERMINAL 2** and **TERMINAL 3**, to stop the `robot_odometry.py` and `turtlebot3_teleop` nodes.  Then, close down **TERMINAL 3** so that only one Windows Terminal application remains open with 2 active tabs: **TERMINAL 1** and **TERMINAL 2**.

<a name="odometry" />

### What is Odometry?

We can learn more about Odometry data by using the `rostopic info` command:

    $ rostopic info /odom

This provides information about the *type* of message used on this topic:

    Type: nav_msgs/Odometry  

We can find out more about this using the `rosmsg info` command:

    rosmsg info nav_msgs/Odometry

Which tells us that the `nav_msgs/Odometry` message contains four *base* elements:

1. header
1. child_frame_id
1. pose
1. twist

**pose** tells us the *position* and *orientation* of the robot relative to an arbitrary reference point (typically where the robot was when it was turned on). The pose is determined from:

* Data from the Inertial Measurement Unit (IMU) onboard the OpenCR board,
* Data from both the left and right wheel encoders,
* An *estimation* of the distance travelled by the robot from its pre-defined reference point (using dead-reckoning).

*Position* data is important for determining the movement of our robot, and from this we can estimate its location in 3-dimensional space.

*Orientation* is expressed in units of [Quaternions](https://en.wikipedia.org/wiki/Quaternion), and needs to be converted into angles (in degrees) about the principal axes. Fortunately, there are functions within the ROS `tf` library to do that for us, which we can use in any Python node as follows:

```python
from tf.transformations import euler_from_quaternion

(roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')
```

Our TurtleBot3 robot can only move in a 2D plane and so, actually, its pose can be fully represented by <code>(x,y,&#952;<sub>z</sub>)</code>, where `x` and `y` are the 2D coordinates of the robot in the `X-Y` plane, and <code>&#952;<sub>z</sub></code> is the angle of the robot about the `z` (*yaw*) axis.  **You should have noticed this in the exercise above, where the `linear_z`, `theta_x` and `theta_y` values in the `delta` column should all have read `0.000` (or changed very little)**.

**twist** tells us the current linear and angular velocities of the robot, and this data comes directly from the wheel encoders.

All of this data is defined in terms of the principal axes, as illustrated in [the figure above](#fig_principal_axes).

<a name="ex2" />

#### Exercise 2: Creating a Python node to process Odometry data

In the previous session you learnt how to create a package and build simple nodes in Python to publish and subscribe to messages on a topic.

1. Create a package [in the same way as last week](Week-1#ex5), this time called `week2_navigation`, which depends on the `rospy`, `nav_msgs` and `geometry_msgs` libraries. Use the `catkin_create_pkg` tool as you did last week. Remember to ensure that you are located in the `~/catkin_ws/src/` directory before you do this though:

        [TERMINAL 2] $ cd ~/catkin_ws/src/
        [TERMINAL 2] $ catkin_create_pkg ...

1. Run `catkin build` on this:

        [TERMINAL 2] $ catkin build week2_navigation

    and then re-source your environment by entering:

        [TERMINAL 2] $ src

1. Navigate to the `src` folder within your `week2_navigation` package using the Linux `cd` command.
1. The `subscriber.py` code that you used last week can be used as a template for creating your odometry subscriber. First, create a new file in your `week2_navigation` package `src` folder (`~/catkin_ws/src/week2_navigation/src`) called `odom_subscriber.py`:

        [TERMINAL 2] $ touch odom_subscriber.py

1. [In the same way as last week](Week-1#chmod), make this file executable using the Linux `chmod` command.
1. In the VS Code File Explorer navigate to the `~/catkin_ws/src/week2_navigation/src` folder and open the `odom_subscriber.py` file that you have just created. Then, copy the [subscriber code from last week](Week-1-Subscriber-Node).

1. Now, edit the code to *subscribe to* and *print out* odometry data to the terminal:
    * You will need to make sure that you are importing the correct message type at the start of your code so that you can work with the Odometry data. In the Week 1 Subscriber we were working with a `String` type message from the `std_msgs` package, whereas this time we need to use an `Odometry` message from the `nav_msgs` package instead. If you need help, have a look at [this explainer](Week-2-Odometry-Explained). 
    * Your Python node should convert the raw odometry data to a <code>(x,y,&#952;<sub>z</sub>)</code> format using the `euler_from_quaternion` function from the `tf.transformations` library (remember that <code>&#952;<sub>z</sub></code> is the same as *Yaw*).  If you aren't sure how to do this, why not have a look at the source code for the `robot_odometry.py` node from the `com2009_examples` package that you used in the [previous exercise](#ex1).  Remember that you can find out where this package is located by using the `roscd` command.
    * You should aim for the output of your node to look something like this: 

        <p align="center">
          <img src="figures/wk02/week2_odom_subscriber.gif">
        </p>

1. Launch your node using `rosrun` and observe how the output (the formatted odometry data) changes whilst you move the robot around again using the `turtlebot3_teleop` node in a new terminal instance (**TERMINAL 3**).
1. Stop your `odom_subscriber.py` node in **TERMINAL 2** and the `turtlebot3_teleop` node in **TERMINAL 3** by entering `Ctrl+C` in each of the terminals.

## Basic Navigation: Open-loop Velocity Control

<a name="ex3" />

#### Exercise 3: Moving a Robot with `rostopic` in the Terminal

> **Note:** *Make sure that you have stopped the `turtlebot3_teleop` node running in **TERMINAL 3** (by entering `Ctrl+C`) before starting this exercise.*

<a name="rostopic_pub" />

We can use the `rostopic pub` command to *publish* data to a topic from a terminal by using the command in the following way:

    rostopic pub {topic_name} {message_type} {data}

As we discovered earlier, the `/cmd_vel` topic is expecting *linear* and *angular* data, each with an `x`, `y` and `z` component. We can get further help with formatting this message by using the autocomplete functionality within the terminal. *Type* the following into **TERMINAL 3** (copying and pasting won't work):

    [TERMINAL 3] rostopic pub /cmd_vel geometry_msgs/Twist[SPACE][TAB]

The full message should then be presented to us:

    $ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

1. Scroll back through the message using the &larr; key on your keyboard and then edit the values of the various parameters, as appropriate. First, define some values that would make the robot *rotate on the spot*.  **Make a note of the command that you used**.
1. Enter `Ctrl+C` in **TERMINAL 3** to stop the message from being published.
1. Next, enter a command in **TERMINAL 3** to make the robot *move in a circle*.  **Again, make a note of the command that you used**.
1. Enter `Ctrl+C` in **TERMINAL 3** to again stop the message from being published.
1. Finally, enter a command to *stop* the TurtleBot **and make a note of this too**.
1. Enter `Ctrl+C` in **TERMINAL 3** to stop this final message from being published.

<a name="ex4" />

#### Exercise 4: Creating a Python node to make the robot move

You will now create another node to control the motion of your TurtleBot3 by publishing messages to the `/cmd_vel` topic. You created a publisher node in Week 1, and you can use this as a starting point.

1. In **TERMINAL 2**, ensure that you are still located within the `src` folder of your `week2_navigation` package. You could use `pwd` to check your current working directory, where the output should look like this:

        /home/student/catkin_ws/src/week2_navigation/src  

    If you aren't located here then navigate to this directory using `cd`.
1. Create a new file called `move_circle.py`:

        [TERMINAL 2] $ touch move_circle.py

    ... and make this file executable using the `chmod` command.
1. Open up this file in VS Code, then copy and paste the contents of [the publisher node from last week](Week-1-Publisher-Node) into the new `move_circle.py` file to get you started. Then edit the code to achieve the following:
    * Make your TurtleBot3 move in a circle with a path radius of approximately 0.5m.
    * The Python node needs to publish `Twist` messages to the `/cmd_vel` topic in order to make the TurtleBot move. Have a look at [this usage example](Week-2-Twist-Example).
    * Remember (as mentioned [earlier](#tb3_max_vels)) that for our robot, the maximum linear velocity (`linear.x`) is 0.26 m/s, and the maximum angular velocity (`angular.z`) is 1.82 rad/s. 
    * Make sure that you code your `shutdownhook()` correctly so that the robot stops moving when the node is shutdown (via `Ctrl+C` in the terminal that launched it).

**Advanced features:**

1. Create a launch file to launch this *and* your `odom_subscriber.py` node simultaneously with a single `roslaunch` command. Refer to the launch file that you created [last week](Week-1#ex8) for a reminder on how to do this.

## Wrapping Up

In this session you have learnt how to control the velocity and position of a robot from both the command-line (using ROS command-line tools) and from ROS Nodes by publishing correctly formatted messages to the `/cmd_vel` topic.  

You have also learnt about *Odometry*, which is published by our robot to the `/odom` topic.  The odometry data tells us the current linear and angular velocities of our robot in relation to its 3 principal axes.  In addition to this though, it also tells us where in physical space our robot is located and oriented, which is determined based on *dead-reckoning*.  You will learn a bit more about dead-reckoning in your lectures.

Consider the following: 
* **How is dead-reckoning implemented, and what information is needed to do this?**
* **What might the implications be for a robot that implements odometry-based navigation?**
* **Can a control method that uses this as a feedback signal be considered an implementation of closed-loop control?** 

We'll explore this a little more next week, but you might want to consider reading Chapter 11.1.3 ("Pose of Robot") in the ROS Robot Programming eBook that we mentioned [here](Home#tb3_ebook).

### Saving your work

Remember, the work you have done in this WSL-ROS environment today **will not be preserved** for future sessions or across different University machines automatically!  To save the work you have done here today you should now run the following script in any idle WSL-ROS Terminal Instance:

    $ wsl_ros backup

<p align="center">
  <strong>Navigating This Wiki:</strong><br />
  <a href="Week-1">&#8592; Week 1: ROS and Linux Basics <strong>[Previous]</strong></a> |
  <a href="Week-3"><strong>[Next]</strong> Week 3: Advanced Navigation and SLAM &#8594;</a>
</p>