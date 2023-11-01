---  
title: "Part 2: Odometry & Navigation"  
description: In this session you'll learn about Odometry data, which informs us of a robot's position in an environment. You'll also learn how to control a ROS robot's velocity (and thus its position) using both open and closed-loop control methods.  
---

## Introduction

:material-pen: **Exercises**: 5  
:material-timer: **Completion Time (approximate)**: 3 hours

### Aims

In Part 2 you will learn how to control a ROS robot's **position** and **velocity** from both the command line and through ROS Nodes. You will also learn how to interpret the data that allows us to monitor a robot's position in its physical environment (odometry).  The things you will learn here form the basis for all robot navigation in ROS, from simple open-loop methods to more advanced closed-loop control (both of which you will explore).

### Intended Learning Outcomes

By the end of this session you will be able to:

1. Interpret the Odometry data published by a ROS Robot and identify the parts of these messages that are relevant to a 2-wheeled differential drive robot (such as the TurtleBot3).
1. Develop Python nodes to obtain Odometry messages from an active ROS network and *translate* them to provide useful information about a robot's *pose* in a convenient, human-readable way.
1. Implement *open-loop velocity control* of a robot using ROS command-line tools.
1. Develop Python nodes that use open-loop velocity control methods to make a robot follow a pre-defined motion path.
1. Combine both publisher *&* subscriber communication methods into a single Python node to implement closed-loop (odometry-based) velocity control of a robot.
1. Explain the limitations of Odometry-based motion control methods. 

### Quick Links

* [Exercise 1: Exploring Odometry Data](#ex1)
* [Exercise 2: Creating a Python node to process Odometry data](#ex2)
* [Exercise 3: Moving a Robot with `rostopic` in the Terminal](#ex3)
* [Exercise 4: Creating a Python node to make the robot move in a circle](#ex4)
* [Exercise 5: Making your robot follow a Square motion path](#ex5)

### Additional Resources

* [Working with Twist Messages in Python](twist-tips)
* [The `move_square` Template (for Exercise 5)](move_square)

## Getting Started

### Step 1: Launch WSL-ROS  
If you haven't done so already, launch your WSL-ROS environment by running the WSL-ROS shortcut in the Windows Start Menu. As you will now know, this may take a couple of minutes, but once it's ready this will open up the Windows Terminal and an *Ubuntu terminal instance* (which we'll refer to as **TERMINAL 1**).

### Step 2: Restore your work  
Remember that any work that you do within the WSL-ROS Environment will not be preserved between sessions or across different University computers.  At [the end of Part 1](../part1/#backup) you should have run the `wsl_ros` tool to back up your home directory to your University U: Drive. Once WSL-ROS is up and running, you should be prompted to restore this:

<figure markdown>
  ![](../../images/wsl/restore_prompt.png){width=600}
</figure>

Enter `Y` to restore your work from last time. You can also restore your work at any time using the following command:

```bash
wsl_ros restore
```

### Step 3: Launch VS Code  
It's also worth launching VS Code now, so that it's ready to go for when you need it later on. [Follow the steps here to launch it correctly](../../../wsl-ros/vscode/).

### Step 4: Download The Course Repo {#course-repo}

We've put together a few ROS packages of our own that you'll use throughout this course. These all live within [this GitHub repo](https://github.com/tom-howard/tuos_ros), and you'll need to download this into the WSL-ROS environment now, before going any further.

1. In **TERMINAL 1**, navigate to the Catkin Workspace `src` directory using the `cd` command:

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/catkin_ws/src/
    ```
    ***

1. Then, clone the Course Repo from GitHub:


    ***
    **TERMINAL 1:**
    ```bash
    git clone https://github.com/tom-howard/tuos_ros.git
    ```
    ***

1. Once this is done, we need to run `catkin build` to compile everything:

    ***
    **TERMINAL 1:**
    ```bash
    catkin build
    ```
    ***

1. And finally, we need to re-source our `.bashrc` file:

    ***
    **TERMINAL 1:**
    ```bash
    source ~/.bashrc
    ```
    ***

    !!! warning "Remember"
        If you have any other terminal instances open, then you'll need run `source ~/.bashrc` in these too, in order for the changes made by `catkin build` to propagate through to these as well!
    
That's it for now, we'll start using some of the packages that we've just installed a bit later on...

### Step 5: Launch the Robot Simulation

In **TERMINAL 1** enter the following command to launch a simulation of a TurtleBot3 Waffle in an empty world:  
        
***
**TERMINAL 1:**
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
***

A Gazebo simulation window should open and within this you should see a TurtleBot3 Waffle in empty space:

<figure markdown>
  ![](../../images/gazebo/tb3_empty_world.png){width=800}
</figure>

<p align="center"><strong><em>You're all set up and ready to go!</em></strong></p>

## Position and Velocity

Two types of **Velocity Command** can be issued to any ROS Robot to make it move (and thus change its *position*):

* **Linear** Velocity: The velocity at which the robot moves *forwards* or *backwards* in one of its principal axes.
* **Angular** Velocity: The velocity at which the robot *rotates* about one of its principal axes.

### Principal Axes {#principal-axes}

The motion (i.e. the velocity) of any mobile robot can be defined in terms of **three** principal axes: `X`, `Y` and `Z`. In the context of our TurtleBot3 Waffle, these axes (and the motion about them) are as follows:

<figure markdown>
  ![](../../images/waffle/principal_axes.svg?width=20cm)
</figure>

In theory then, a robot can move linearly or angularly about *any* of these three axes, as shown by the arrows in the figure. That's six *Degrees of Freedom* (DOFs) in total, achieved based on a robot's design and the actuators it is equipped with. 

You should hopefully recall from the ["Introducing the Robots" page](../../../about/robots/#tb3) that our TurtleBot3 Waffles only have two motors though, so they don't actually have six DOFs! These two motors can be controlled independently, which is known as a *"differential drive"* configuration, and ultimately provides it with a total of **two degrees of freedom** in total, as illustrated below.

<figure markdown>
  ![](../../images/waffle/velocities.svg?width=20cm)
</figure>

It can therefore only move **linearly** in the **x**-axis (*Forwards/Backwards*) and **angularly** in the **z**-axis (*Yaw*).

It's also worth noting (while we're on the subject of motion) that our TurtleBot3 Waffles have **maximum velocity limits**, which were also defined on the ["Robots"](../../../about/robots/#tb3) page.

!!! question
    What are the maximum velocity limits of our robots?

### ROS Velocity Commands

In Part 1 you learnt how to [list all the topics that are currently active on a ROS system](../part1/#rostopic). Open up a new terminal instance now (**TERMINAL 2**) and use what you learnt previously to *list* all the topics that are active on the ROS network now, as a result of launching the Gazebo simulation earlier.

!!! question "Questions"
    1. Which topic in the list do you think could be used to control the velocity of the robot?
    2. Use the `rostopic info` command on the topic to find out more about it.

The topic you identified[^cmd_vel] should use a message of the `geometry_msgs/Twist` type. You'll have to send messages of this type to this topic in order to make the robot move. Use the `rosmsg` command ([as you did in Part 1](../part1/#ex3)) to find out more about the format of this message[^rosmsg_info].

[^cmd_vel]: The topic is `/cmd_vel` (i.e. *command velocity*).
[^rosmsg_info]: Answer: `rosmsg info geometry_msgs/Twist`.

You should now be looking at a message format that looks like this: 

    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z

There are **six** parameters that we can assign values to here: 

1. `linear.x`
1. `linear.y`
1. `linear.z`
1. `angular.x`
1. `angular.y`
1. `angular.z`

These relate to a robot's **six degrees of freedom** (about its three principal axes), as we discussed above. These topic messages are therefore formatted to give a ROS Programmer the ability to *ask* a robot to move in any one of its six DOFs. 

    geometry_msgs/Vector3 linear
      float64 x  <-- Forwards (or Backwards)
      float64 y  <-- Left (or Right)
      float64 z  <-- Up (or Down)
    geometry_msgs/Vector3 angular
      float64 x  <-- Roll
      float64 y  <-- Pitch
      float64 z  <-- Yaw

As we also learnt above though, our TurtleBots can only actually move with **linear** velocity in the **x**-axis and **angular** velocity in the **z**-axis. As a result then, only velocity commands issued to the `linear.x` (Forwards/Backwards) or `angular.z` (Yaw) parts of this message will have any effect.

### Robot Odometry

Another topic that should have appeared when you ran `rostopic list` earlier is `/odom`. This topic contains *Odometry data*, which is also essential for robot navigation and is a basic feedback signal, allowing a robot to approximate its location.

#### :material-pen: Exercise 1: Exploring Odometry Data {#ex1}

1. In **TERMINAL 2** use the `rostopic echo` command to display the odometry data currently being published by our simulated robot:

    ***
    **TERMINAL 2:**
    ```bash
    rostopic echo -c /odom
    ```
    ***

    Expand the terminal window as necessary so that you can see the whole topic message (it starts with `header` and ends with `---`).
    
    !!! question
        What does the `-c` option in the command above actually do?
    
1. Now, you need to launch a new Windows Terminal instance so that you can view it side-by-side with **TERMINAL 2**. To do this, press the "New Tab" button whilst pressing the ++shift++ key. We'll call this one **TERMINAL 3**. Arrange both windows side-by-side, so you can see what's happening in both, simultaneously.

1. In **TERMINAL 3** launch the `turtlebot3_teleop_keyboard` node [as you did last time](../part1/#ex1): <a name="teleop"></a>

    ***
    **TERMINAL 3:**
    ```bash
    rosrun turtlebot3_teleop turtlebot3_teleop_key
    ```
    ***

1. In **TERMINAL 3** enter ++a++ a couple of times to make the robot rotate on the spot.  Observe how the odometry data changes in **TERMINAL 2**.  Is there anything in the `twist` part of the `/odom` message that corresponds to the "angular vel" that you are setting in **TERMINAL 3**? 
1. Now press the ++s++ key to halt the robot, then press ++w++ a couple of times to make the robot drive forwards.  How does the `twist` part of the message now correspond to the "linear vel" setting in **TERMINAL 3**?
1. Now press ++d++ a couple of times and your robot should start to move in a circle.  What linear and angular velocities are you requesting in **TERMINAL 3**, and how are these represented in the `twist` part of the `/odom` message?  What about the `pose` part of the message?  How is this data changing as your robot moves in a circular path.
    
    !!! question
        What do you think `twist` and `pose` are actually telling us?
    
1. Press ++s++ in **TERMINAL 3** to stop the robot (but leave the `turtlebot3_teleop_key` node running).  Then, press ++ctrl+c++ in **TERMINAL 2** to shut down the `rostopic echo` process. 

1. Let's look at the `pose` part of the `Odometry` message in more detail now. With the robot stationary, use `rosrun` to run a Python node that we have created to help illustrate how this relates to the robot's position and orientation in its environment: 

    ***
    **TERMINAL 2:**
    ```bash
    rosrun tuos_examples robot_pose.py
    ```
    ***
        
1. Now (using the `turtlebot3_teleop_key` node in **TERMINAL 3**), drive your robot around again, keeping an eye on the outputs that are being printed by the `robot_pose.py` node in **TERMINAL 2** as you do so.

    The output of the `robot_pose.py` node shows you how the robot's *position* and *orientation* (i.e. *"pose"*) are changing in real-time as you move the robot around. The `"initial"` column tells us the robot's pose when the node was first launched, and the `"current"` column show us what its pose currently is. The `"delta"` column then shows the difference between the two.
    
    !!! question
        Which pose parameters *haven't* changed, and is this what you would expect (considering [the robot's principal axes, as illustrated above](#principal-axes))?
    
1. Press ++ctrl+c++ in **TERMINAL 2** and **TERMINAL 3**, to stop the `robot_pose.py` and `turtlebot3_teleop_key` nodes.  Then, close down **TERMINAL 3** so that only one Windows Terminal application remains open with 2 active tabs: **TERMINAL 1** and **TERMINAL 2**.

### What is Odometry? {#odometry}

We can learn more about Odometry data by using the `rostopic info` command:

***
**TERMINAL 2:**
```bash
rostopic info /odom
```
***

This provides information about the *type* of message used by this topic:

    Type: nav_msgs/Odometry  

We can find out more about this using the `rosmsg info` command:

***
**TERMINAL 2:**
```bash
rosmsg info nav_msgs/Odometry
```
***

Which tells us that the `nav_msgs/Odometry` message contains four *base* elements:

1. header
1. child_frame_id
1. pose
1. twist

#### Pose

**Pose** tells us the *position* and *orientation* of the robot relative to an arbitrary reference point (typically where the robot was when it was turned on). The pose is determined from:

* Data from the Inertial Measurement Unit (IMU) on the OpenCR board,
* Data from both the left and right wheel encoders,
* An *estimation* of the distance travelled by the robot from its pre-defined reference point (using dead-reckoning).

*Position* data is important for determining the movement of our robot, and from this we can estimate its location in 3-dimensional space.

<a name="euler_angs"></a>*Orientation* is expressed in units of [Quaternions](https://en.wikipedia.org/wiki/Quaternion), and needs to be converted into Euler angles (in radians) about the principal axes. Fortunately, there are functions within the ROS `tf` library to do that for us, which we can use in any Python node as follows:

```python
from tf.transformations import euler_from_quaternion

(roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')
```

Our TurtleBot3 can only move in a 2D plane and so, actually, its pose can be fully represented by 3 parameters: <code>(x,y,&theta;<sub>z</sub>)</code>, where `x` and `y` are the 2D coordinates of the robot in the `X-Y` plane, and <code>&theta;<sub>z</sub></code> is the angle of the robot about the `z` (*yaw*) axis.

!!! question
    In the previous exercise, did you notice how the `linear_z`, `theta_x` and `theta_y` values in the `delta` column all remained at `0.000`, even when the robot was moving around?

#### Twist

**Twist** tells us the current linear and angular velocities of the robot, and this data comes directly from the wheel encoders.

Once again, all of this data is defined in terms of the principal axes, as illustrated in [the figure above](#principal-axes).

#### :material-pen: Exercise 2: Creating a Python node to process Odometry data {#ex2}

In Part 1 you learnt how to create a package and build simple nodes in Python to publish and subscribe to messages on a topic. In this exercise you will build a new subscriber node, much like you did in the previous session, but this one will subscribe to the `/odom` topic that we've been talking about above. You'll also create a new package called `part2_navigation` for this node to live in!

1. Create a package [in the same way as you did in Part 1](../part1/#ex4), this time called `part2_navigation`, which depends on the `rospy`, `nav_msgs` and `geometry_msgs` libraries. Use the `catkin_create_pkg` tool as you did in Part 1. Remember to ensure that you are located in the `~/catkin_ws/src/` directory before you do this though:
     
    ***
    **TERMINAL 2:**
    ```bash
    cd ~/catkin_ws/src/
    ```
    Then:
    ```bash
    catkin_create_pkg part2_navigation {BLANK}
    ```
    ***

    !!! warning "Fill in the Blank!"
        Recall how we used the `catkin_create_pkg` tool [in Part 1](../part1/#ex4), but adapt this now for the `part2_navigation` package, as detailed above.
    
1. Run `catkin build` on this:

    ***
    **TERMINAL 2:**
    ```bash
    catkin build part2_navigation
    ```
    and then re-source your environment:
    ```bash
    source ~/.bashrc
    ```
    ***

1. The subscriber that we will build here will be structured in much the same way as [the subscriber that we built in Part 1](../part1/subscriber). The difference now though is that this one will subscribe to the `/odom` topic (instead of `/chatter`), and its callback function will therefore receive `Odometry` type messages (instead of `String`), so we'll have to deal with those a bit differently. We've created a template for this to help you to get started. Download this into the `src` directory of your new `part2_navigation` package now:
    
    ***
    **TERMINAL 2:**

    1. Step 1: navigate to the `src` directory of your `part2_navigation` package:
        ```bash
        cd ~/catkin_ws/src/part2_navigation/src/
        ```
    1. Then download the template code from GitHub:
        ```bash
        wget -O odom_subscriber.py https://raw.githubusercontent.com/tom-howard/tuos_ros/main/tuos_examples/src/odom_subscriber_template.py
        ```
    1. Finally, make this executable using `chmod`:
        ```bash
        chmod +x odom_subscriber.py
        ```
    ***

1. Run this as it is to see what happens to begin with:

    ***
    **TERMINAL 2:**
    ```bash
    rosrun part2_navigation odom_subscriber.py
    ```
    ***

    ... Hmmm, something's wrong here isn't it!? You may have seen the following error:

    ```txt
    /usr/bin/env: ‘python3\r’: Permission denied
    ```

    The clue here is the `python3\r` (specifically the `\r` bit). This is a *Windows line ending*... 
    
    Text files (including things like Python scripts) created on Windows use different line endings (i.e. the characters that signify the end of each line of text) to those created on Linux. Windows uses a "carriage return" *and* a "line feed" (`\r\n`) at the end of each line, but Linux uses just a "line feed" (`\n`)[^source]. Because we're working within a Linux environment here (Ubuntu), we must make sure we're using Linux line endings at all times! We can change this easily from inside VS Code... 
    
    [^source]: Adapted from: https://www.cs.toronto.edu/~krueger/csc209h/tut/line-endings.html

    1. In the VS Code File Explorer navigate to the `~/catkin_ws/src/part2_navigation/src` folder and open the `odom_subscriber.py` file.
    1. In the blue bar along the bottom of the VS Code screen (towards the bottom right-hand corner) you should see the text `CRLF`. Click on this and a menu should then appear at the top of the screen with the text `"Select End of Line Sequence"`.
    1. Select the `LF` option in this menu, then save the file.

    <figure markdown>
      ![](../../images/vscode/switch_line_ending.png?width=800px)
    </figure> 

1. OK, the file should run now, so launch it (using `rosrun` again) and see what it does.

1. Have a think about what's different between this and [the subscriber from last time](../part1/subscriber)...
    
    In the Subscriber from Part 1 we were working with a `String` type message from the `std_msgs` package, whereas this time we're using an `Odometry` message from the `nav_msgs` package instead - notice how the imports and the callback function have changed as a result of this.

1. You need to add some additional code to the callback function now: 
    1. The node needs to print the robot's real-time odometry data to the terminal in the form: <code>(x,y,&#952;<sub>z</sub>)</code>.
    1. The format of the message has already been structured for you, but you need to add in the relevant variables that represent the correct elements of the robot's real-time pose.
    1. You'll need to use the `euler_from_quaternion` function from the `tf.transformations` library to convert the raw orientation values from Quaternions into Radians. If you need a hint, why not have a look back [at this bit from earlier](#euler_angs), or at the source code for the `robot_pose.py` node that we launched from the `tuos_examples` package in the [previous exercise](#ex1). 

1. Launch your node using `rosrun` and observe how the output (the formatted odometry data) changes whilst you move the robot around again using the `turtlebot3_teleop_key` node in a new terminal instance (**TERMINAL 3**).
1. Stop your `odom_subscriber.py` node in **TERMINAL 2** and the `turtlebot3_teleop` node in **TERMINAL 3** by entering ++ctrl+c++ in each of the terminals.

## Basic Navigation: Open-loop Velocity Control

#### :material-pen: Exercise 3: Moving a Robot with `rostopic` in the Terminal {#ex3}

!!! warning
    Make sure that you've stopped the `turtlebot3_teleop_key` node running in **TERMINAL 3** (by entering ++ctrl+c++) before starting this exercise.

<a name="rostopic_pub"></a>We can use the `rostopic pub` command to *publish* data to a topic from a terminal by using the command in the following way:

```bash
rostopic pub {topic_name} {message_type} {data}
```

As we discovered earlier, the `/cmd_vel` topic is expecting *linear* and *angular* data, each with an `x`, `y` and `z` component. We can get further help with formatting this message by using the autocomplete functionality within the terminal. Type the following into **TERMINAL 3** hitting the ++space++ and ++tab++ keys on your keyboard as indicated below:

***
**TERMINAL 3:**
```bash
rostopic pub /cmd_vel geometry_msgs/Twist[SPACE][TAB]
```

The full message should then be presented to us:

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

***

1. Scroll back through the message using the &larr; key on your keyboard and then edit the values of the various parameters, as appropriate. First, define some values that would make the robot **rotate on the spot**.  Make a note of the command that you used.
1. Enter ++ctrl+c++ in **TERMINAL 3** to stop the message from being published.
1. Next, enter a command in **TERMINAL 3** to make the robot **move in a circle**.  Again, make a note of the command that you used.
1. Enter ++ctrl+c++ in **TERMINAL 3** to again stop the message from being published.
1. Finally, enter a command to **stop** the TurtleBot3 and make a note of this too.
1. Enter ++ctrl+c++ in **TERMINAL 3** to stop this final message from being published.

#### :material-pen: Exercise 4: Creating a Python node to make the robot move in a circle {#ex4}

You will now create another node to control the motion of your TurtleBot3 by publishing messages to the `/cmd_vel` topic. You created a publisher node in Part 1, and you can use this as a starting point.

1. In **TERMINAL 2**, ensure that you are still located within the `src` folder of your `part2_navigation` package. You could use `pwd` to check your current working directory, where the output should look like this:

        /home/student/catkin_ws/src/part2_navigation/src  

    If you aren't located here then navigate to this directory using `cd`.

1. Create a new file called `move_circle.py`:

    ***
    **TERMINAL 2:**
    ```bash
    touch move_circle.py
    ```
    ... and make this file executable using the `chmod` command.
    ***

1. Open up this file in VS Code. This node should make the TurtleBot3 move in a **circle** with a path **radius** of approximately **0.5 meters**: <a name="ex4_ret"></a>

    * Your Python node needs to publish `Twist` messages to the `/cmd_vel` topic in order to make the TurtleBot3 move. [See here for some tips on this](twist-tips).
    * Remember that our robots have a maximum linear velocity (`linear.x`) of 0.26 m/s, and a maximum angular velocity (`angular.z`) of 1.82 rad/s. 
    * Make sure that you code your `shutdownhook()` correctly so that the robot stops moving when the node is shutdown (via ++ctrl+c++ in the terminal that launched it).

    Use this code template to get you started:

    ```python title="A template for the move_circle.py node"
    --8<-- "snippets/move_circle.py"
    ```
    
    1. This is important, we always need to import `rospy`
    2. What other imports might we need here in order to create and publish a message to make the robot move?
    3. Give your node a descriptive name - this is the name that it will be given when it is registered on the ROS network, and the one that you would see if you used the `rosnode list` command.
    4. What do you need to add here in order to set up an appropriate publisher to the `/cmd_vel` topic?
    5. Define an appropriate rate for your `main()` loop to run at.
    6. Anything in here will run when the node receives a shutdown request (i.e., we enter ++ctrl+c++ in the terminal). What actions would be important to take here to make sure the node shuts down safely and the robot actually stops moving?
    7. You're going to need to create a message here containing appropriate velocities for the robot to move at. Then you'll need to actually publish that message to `/cmd_vel` (via your `self.pub` object). Finally, how would you use the `self.rate` object that you created above to control the execution rate of the while loop?
    8. What will you need to do here to instantiate your `Circle()` class and execute its main functionality?

    Refer back to [the publisher node from Part 1](../part1/publisher) to help you as you're working on this. 

**Advanced feature:**

1. Create a launch file to launch this *and* your `odom_subscriber.py` node simultaneously with a single `roslaunch` command. Refer to the launch file that you created [in Part 1](../part1/#ex8) for a reminder on how to do this.

## Odometry-based Navigation

In the previous exercise you created a Python node to make your robot move using *open-loop control*. To achieve this you published velocity commands to the `/cmd_vel` topic to make the robot follow a circular motion path.

!!! question "Questions"
    1. How do you know if your robot actually achieved the motion path that you were hoping for?
    1. In a real-world environment, what external factors might result in your robot *not* achieving its desired trajectory?

Earlier on you also learnt about [Robot Odometry](#odometry), which is used by the robot to keep track of its **position** and **orientation** (aka **Pose**) in the environment.  This is determined by a process called *"dead-reckoning,"* which is only really an approximation, but it's a fairly good one in any case, and we can use this as a feedback signal to understand if our robot is moving in the way that we expect it to.  We can therefore build on the techniques that we used in the `move_circle.py` exercise, and now also build in the ability to *subscribe* to a topic too. In this case, we'll be subscribing to the `/odom` topic that we worked with a bit (in isolation) in [Exercise 2](#ex2), and use this to provide us with a feedback signal to allow us to implement some basic *closed-loop control*.

#### :material-pen: Exercise 5: Making your robot follow a Square motion path {#ex5}

1. Make sure your `move_circle.py` node is no longer running in **TERMINAL 2**, stopping it with ++ctrl+c++ if necessary.
1. Make sure **TERMINAL 2** is still located inside your `part2_navigation` package[^roscd].
    
    [^roscd]: Remember, you can use the `roscd` command for this!
    
1. Navigate to the package `src` directory and use the Linux `touch` command to create a new file called `move_square.py`:
    
    ***
    **TERMINAL 2:**
    ```bash
    touch move_square.py
    ```
    ***

1. Then make this file executable using `chmod`:

    ***
    **TERMINAL 2:**
    ```bash
    chmod +x move_square.py
    ```
    ***

1. Use the VS Code File Explorer to navigate to this `move_square.py` file and open it up, ready for editing.
1. [There's a template here to help you with this exercise](move_square). Copy and paste the template code into your new `move_square.py` file to get you started. <a name="ex5_ret"></a>
1. Run the code as it is to see what happens...

    !!! warning "Fill in the Blank!"
        Something not quite working as expected? We may have missed out [something very crucial](../part1/subscriber/#dfts) on **the very first line** of the code template, can you work out what it is?!

1. Fill in the blank as required and then adapt the code to make your robot follow a **square** motion path of **1 x 1 meter** dimensions:
    * The robot's odometry will tell you how much the robot has moved and/or rotated, and so you should use this information to achieve the desired motion path. 
    * Your Python node will therefore need to *subscribe* to the `/odom` topic as well as *publish* to `/cmd_vel`.

**Advanced features:**

1. Adapt the node further to make the robot automatically stop once it has performed two complete loops.
1. Create a launch file to launch this *and* the `odom_subscriber.py` node from last time simultaneously!

After following a square motion path a few times, your robot *should* return to the same location that it started from.

## Wrapping Up

In this session you have learnt how to control the velocity and position of a robot from both the command-line (using ROS command-line tools) and from ROS Nodes by publishing correctly formatted messages to the `/cmd_vel` topic.  

You have also learnt about *Odometry*, which is published by our robot to the `/odom` topic.  The odometry data tells us the current linear and angular velocities of our robot in relation to its 3 principal axes.  In addition to this though, it also tells us where in physical space our robot is located and oriented, which is determined based on *dead-reckoning*. 

!!! question "Questions" 
    1. If odometry is derived from dead-reckoning, what information (sensor/actuator data) is used to do this?
    1. Do you see any potential limitations of this?
    1. Can a control method that uses odometry as a feedback signal be considered *closed-loop control?* 

Consider reading Chapter 11.1.3 ("Pose of Robot") in the ROS Robot Programming eBook that we mentioned [here](../../../about/robots/#ebook).

In the final exercise you also learnt how to develop an odometry-based controller to make your robot follow a square motion path. You will likely have observed some degree of error in this which, as you already know, could be due to the fact that Odometry data is determined by dead-reckoning and is therefore subject to drift and error.  Consider how other factors may impact the accuracy of control too:

!!! question "Questions"
    1. How might the rate at which the odometry data is sampled play a role?
    1. How quickly can your robot receive new velocity commands, and how quickly can it respond?

Be aware that we did all this in simulation here too. In fact, in a real world environment, this type of navigation might be *less effective*, since things such as measurement noise and calibration errors can also have considerable impact. You will have the opportunity to experience this first hand later in this course.

Ultimately then, we have seen a requirement here for additional information to provide more confidence of a robot's location in its environment, in order to enhance its ability to navigate effectively and avoid crashing into things! We'll explore this further in the next part of this course.

### Saving your work {#backup}

Remember, the work you have done in the WSL-ROS environment during this session **will not be preserved** for future sessions or across different University machines automatically! To save the work you have done here today you should now run the following script in any idle WSL-ROS Terminal Instance:

```bash
wsl_ros backup
```

This will export your home directory to your University U: Drive, allowing you to restore it at the start of the next session.  
