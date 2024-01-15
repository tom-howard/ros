---  
title: "ROS & Waffle Basics"  
---

# ROS & Waffle Basics

Having completed the steps on [the previous page](../launching-ros), your robot and laptop should now be paired, and ROS should be up and running. The next thing to do is bring the robot to life! 

On this page you'll work through a series of exercises with the TurtleBot3 (aka, the Waffle) **in your teams**, exploring how the robot works whilst also getting an initial insight into how ROS works too. A number of the exercises here are similar to those that you'll do (or perhaps have *already* done) individually in simulation in [Assignment #1](../../com2009/assignment1/). As you'll soon see, whether you're working with a real robot or a simulation, a lot of the principles are the same for both. 

### Quick Links

* [Exercise 1: Making the Robot Move](#exMove)
* [Exercise 2: Cloning Your Team's ROS Package to the Robot Laptop](#exClone)
* [Exercise 3: Seeing the Sensors in Action](#exViz)
* [Exercise 4: Visualising the ROS Network](#exNet)
* [Exercise 5: Exploring ROS Topics and Messages](#exTopicMsg)
* [Exercise 6: Creating Your First Python Node](#exSimpleVelCtrl)
* [Exercise 7: Using SLAM to create a map of the environment](#exSlam)
* [Exercise 8: Pushing Changes to Your ROS Package Back to GitHub](#exGitPush)

## Manual Control

#### :material-pen: Exercise 1: Making the Robot Move {#exMove}

Throughout Lab Assignment #1 you will use a ready-made ROS application called `turtlebot3_teleop_keyboard` to drive a Waffle around a range of simulated environments. This works in exactly the same way with a real robot in a real world too:

1. Open up a new terminal instance on the laptop either by using the ++ctrl+alt+t++ keyboard shortcut, or by clicking the Terminal App icon, we'll refer to this as **TERMINAL 1**. In this terminal enter the following `rosrun` command to launch `turtlebot3_teleop_keyboard` (note that it's exactly [the same command as you use in simulation](../../com2009/assignment1/part1/#teleop) too):

    ***
    **TERMINAL 1:**
    ```bash
    rosrun turtlebot3_teleop turtlebot3_teleop_key
    ```
    
    ??? tip "Robotics Laptop Tip"
        There's a bash alias for this command to make it quicker to type in future: `tb3_teleop`!

    ***

1. Follow the instructions provided in the terminal to drive the robot around using specific buttons on the keyboard:

    <figure markdown>
      ![](../images/ros-cli/teleop_keymap.svg)
    </figure>

    !!! warning 
        Take care to avoid any obstacles or other people in the lab as you do this!

2. Once you've spent a bit of time on this, close the application down by entering ++ctrl+c++ in **TERMINAL 1**.

## Packages and Nodes

ROS applications are organised into *packages*. Packages are basically folders containing scripts, configurations and launch files (ways to launch those scripts and configurations).  

*Scripts* tell the robot what to do and how to act. In ROS, these scripts are called *nodes*. *ROS Nodes* are executable programs that perform specific robot tasks and operations. These are typically written in C++ or Python, but it's possible to write ROS Nodes using other programming languages too.

In the initial setup of the robot on the previous page ([Step 3](../launching-ros/#step-3-launching-ros)) you simultaneously established a ROS Network ("the ROS Master") *and* launched a range of different nodes on the robot with a `roslaunch` command. Then, in [Exercise 1 above](#exMove) you launched the `turtlebot3_teleop_key` node on the laptop:

<center>

|  | Command | Context | 
| :---: | :--- | :---: |
| 1 | `#!sh roslaunch tuos_tb3_tools ros.launch` | Robot |
| 2 | `#!sh rosrun turtlebot3_teleop turtlebot3_teleop_key` | Laptop |

</center>

The first of the above commands was a `roslaunch` command, which has the following two parts to it (after the `roslaunch` bit):

``` { .bash .no-copy }
roslaunch {[1] Package name} {[2] Launch file}
```

**Part [1]** specifies the name of the *ROS package* containing the functionality that we want to execute. **Part [2]** is a file within that package that tells ROS exactly what scripts (*'nodes'*) that we want to launch. We can launch multiple nodes at the same time from a single launch file.

The second command was a `rosrun` command, which has a structure similar to `roslaunch`:

``` { .bash .no-copy }
rosrun {[1] Package name} {[2] Node name}
```    

Here, **Part [1]** is the same as the `roslaunch` command, but **Part [2]** is slightly different: `{[2] Node name}`. Here we are directly specifying a single script that we want to execute. We therefore use `rosrun` if we only want to launch a **single node** on the ROS network (`turtlebot3_teleop_key` in this case, which is a Python script).

The key difference between `roslaunch` and `rosrun` then is that with `roslaunch` we can execute **1 or more** nodes at the same time via *launch files*. Another handy feature of `roslaunch` is that it will automatically launch the **ROS Master** if it isn't already running. As illustrated by the "Context" column in the table above, we ran our `roslaunch` command on the robot, which means that all the specified nodes (defined in the `ros.launch` file[^tuos_ros_launch]) *and* the ROS Master were executed **on the robot**.

[^tuos_ros_launch]: Source code available here (if you're interested): https://github.com/tom-howard/tuos_ros/blob/main/tuos_tb3_tools/launch/ros.launch


!!! info "The ROS Master"
    The ROS Master is a wireless communication network that is established between the robot and laptop (and indeed any other device that we might want to add). The benefit of this then is that it allowed us to run our `rosrun` command on the laptop, and this was able to invoke changes to the robot (i.e. making it move around) via this wireless ROS communication network.

#### :material-pen: Exercise 2: Cloning Your Team's ROS Package to the Robot Laptop {#exClone}

In the Assignment #2 "Getting Started" tasks that you should have completed earlier you should have [created your team's Assignment #2 ROS package](../../com2009/assignment2/getting-started/#create-pkg) and [pushed it to GitHub](../../com2009/assignment2/getting-started/#github). In this exercise you will now clone it on to the Robotics Laptop and create your first Python ROS node within it.

!!! warning "WiFi"
    Remember, the Robotics Laptop needs to be connected to the "DIA-LAB" WiFi network in order for the robot and laptop to communicate with one another, but DIA-LAB is an internal network, and you won't be able to access the internet!
    
    Make sure the laptop is now connected to "eduroam" before starting this exercise.

You'll use SSH keys to download your team's ROS package onto the laptop now. You'll need to follow a similar procedure if you end up working on a different laptop during a different lab, or if you happen to delete your package from the laptop that you're working on now. There are some [more detailed instruction on how all this works here (TODO)](), which you should refer to in future lab sessions.

##### Step 1: Generating an SSH key (on the Laptop) {#ssh-keygen}

1. From a terminal instance on the laptop (i.e. **TERMINAL 1**) navigate to the `~/.ssh` folder using the `cd` Linux Command (*"change directory"*):

    ```bash
    cd ~/.ssh
    ```

1. Create a new SSH key on the laptop, using your GitHub email address:

    ``` { .bash .no-copy}
    ssh-keygen -t ed25519 -C "your.email@sheffield.ac.uk"
    ```

    Replacing `your.email@sheffield.ac.uk` with **your GitHub email address**.

1. You'll then be asked to **"Enter a file in which to save the key"**. This needs to be unique, so enter the name of your ROS package. For the purposes of this example, let's assume yours is called `com2009_team999`.

1. You'll then be asked to **enter a passphrase**. This is how you make your SSH key secure, so that no other teams using the same laptop can access and make changes to your team's package/GitHub repo. You'll be asked to enter this whenever you try to commit/push new changes back to GitHub. Decide on a passphrase and share this with **ONLY YOUR TEAM**. 

1. Next, start the laptop's ssh-agent:

    ```bash
    eval "$(ssh-agent -s)"
    ```

1. Add your SSH private key to the laptop's ssh-agent. You'll need to enter the name of the SSH key file that you created in the earlier step (e.g.: `com2009_team999`)

    ``` { .bash .no-copy}
    ssh-add ~/.ssh/com2009_team999
    ```

    Replacing `com2009_team999` with the name of your own SSH key file.

1. Then, you'll need to add the SSH key to your account on GitHub...

##### Step 2: Adding the SSH key to your GitHub account

1. On the laptop, copy the SSH public key that you created in the previous steps to your clipboard.
    
    Do this from a terminal on the laptop, using `cat`:

    ``` { .bash .no-copy}
    cat ~/.ssh/com2009_team999.pub
    ```

    replacing `com2009_team999` with the name of your SSH key file.

    The content of the file will then be displayed in the terminal... copy it from here.

    !!! tip "Tips"
        1. To copy text from inside a terminal window use ++ctrl+shift+c++
        2. You could also open the file in VS Code and copy it from there:

            ```bas { .bash .no-copy}h
            code ~/.ssh/com2009_team999.pub
            ```

2. Go to your GitHub account in a web browser. In the upper-right corner of any page, click your profile photo, then click **Settings**.

3. In the "Access" section of the sidebar, click **SSH and GPG keys**.

4. Click **New SSH key**.

5. Enter a descriptive name for the key in the "Title" field, e.g. `com2009_dia-laptop1`.

6. Select `Authentication Key` as the "Key Type."

7. Paste the text from your SSH Public Key file into the "Key" field.

8. Finally, click the "Add SSH Key" button.

##### Step 3: Cloning your ROS package onto the Laptop {#ssh-clone}

With your SSH keys all set up, you can now clone your ROS package onto the laptop. 

There's a "Catkin Workspace" on each of the robot laptops and your package **must** reside within this workspace. (You'll learn more about Catkin Workspaces in Assignment #1.)

1. From a terminal on the laptop, navigate to the Catkin Workspace `src` directory:

    ```bash
    cd ~/catkin_ws/src
    ```

1. Go to your ROS package on GitHub. Click the Code button and then select the SSH option to reveal the SSH address of your repo. Copy this. 

1. Head back to the terminal instance on the laptop to then clone your package into the `catkin_ws/src/` directory using `git`:

    ``` { .bash .no-copy}
    git clone {REMOTE_SSH_ADDRESS}
    ```

    Where `{REMOTE_SSH_ADDRESS}` is the SSH address that you have just copied from GitHub.

    !!! tip
        To *paste* text into the Linux terminal window use ++ctrl+shift+v++

1. Run Catkin Build to make sure that any resources within your package that need to be compiled (custom ROS messages, etc.) are compiled onto the laptop so that they can be used locally:
	
    ``` { .bash .no-copy}
    catkin build com2009_team999
    ```
	
	...again, replacing `com2009_team999` with *your* team's package name.
	
1. Then, re-source your environment:
	
    ```bash
    source ~/.bashrc
    ```

    !!! note
        This will all become very familiar with you once you've worked through Assignment #1!

You should then be able to commit and push any updates that you make to your ROS package while working on the laptop back to your remote repository using the secret passphrase that you defined earlier!

## Sensors & Visualisation Tools

!!! warning "WiFi"
        
    Make sure the laptop is now connected back to "DIA-LAB" in order to continue with the rest of the exercises.

Our Waffles have some pretty sophisticated sensors on them, allowing them to "see" the world around them. Let's now see what our robot sees, using some handy ROS tools.

#### :material-pen: Exercise 3: Seeing the Sensors in Action {#exViz}

##### Part 1: The Camera

1. There shouldn't be anything running in **TERMINAL 1** now, after you closed down the Teleop node (using ++ctrl+c++) at the end of the previous exercise. Return to this terminal and launch the `rqt_image_view` node:

    ***
    **TERMINAL 1:**
    ```bash
    rosrun rqt_image_view rqt_image_view
    ```
    ***

    !!! question "Questions"
        1. We're using `rosrun` here again, what does this mean?
        1. Why do we have to type `rqt_image_view` twice?
    
1. A new window should open. Maximise this (if it isn't already) and then select `/camera/color/image_raw` from the dropdown menu at the top-left of the application window.
1. Live images from the robot's camera should now be visible! Stick your face in front of the camera and see yourself appear on the laptop screen!
1. Close down the window once you've had enough. This should release **TERMINAL 1** so that you can enter commands in it again.

    The camera on the robot is quite a clever device. Inside the unit is two separate image sensors, giving it - effectively - both a left and right eye. The device then combines the data from both of these sensors and uses the combined information to infer depth from the images as well. Let's have a look at that in action now...

1. In **TERMINAL 1** enter the following command:

    ***
    **TERMINAL 1:**
    ```bash
    roslaunch tuos_tb3_tools rviz.launch
    ```
    ***
    
    This will launch an application called *RViz*, which is a handy tool that allows us to *visualise* the data from all the sensors on-board our robots. When RViz opens, you should see something similar to the following:

    <figure markdown>
      ![](../images/laptops/waffle_rviz.png?width=20cm)
    </figure>

    The strange wobbly sheet of colours in front of the robot is the live image stream from the camera with depth applied to it at the same time. The camera is able to determine how far away each image pixel is from the camera lens, and then uses that to generate this 3-dimensional representation. Nice eh!

1. Again, place your hand or your face in front of the camera and hold steady for a few seconds (there may be a bit of a lag as all of this data is transmitted over the WiFi network). You should see yourself rendered in 3D in front of the robot! 

##### Part 2: The LiDAR Sensor

In RViz you may have also noticed a lot of red dots scattered around the robot. This is a representation of the *laser displacement data* coming from the LiDAR sensor (the black device on the top of the robot). The LiDAR sensor spins continuously, sending out laser pulses into the environment as it does so. When a pulse hits an object it is reflected back to the sensor, and the time it takes for this to happen is used to calculate how far away the object is.
    
The LiDAR sensor spins and performs this process continuously, so a full 360&deg; scan of the environment can be generated. This data is therefore really useful for things like *obstacle avoidance* and *mapping*. We'll explore this in more detail later.

1. For now, move your hand around the robot and see if you can see it being detected by the LiDAR sensor. Move your hand up and down and consider at what height the LiDAR sensor is able to detect it.

1. Then, move your hand closer and further away and watch how the red dots move to match this. 

1. Open up a new terminal instance (**TERMINAL 2**) and launch the `turtlebot3_teleop_keyboard` node as you did in Exercise 1. Watch how the data in the RViz screen changes as you drive the robot around a bit.

1. Once you've had enough, close down RViz (click the "Close without saving" button, if asked) and stop the Keyboard Teleop node by entering ++ctrl+c++ in **TERMINAL 2**.

Using `rosrun` and `roslaunch`, as we have done so far, it's easy to end up with a lot of different processes or *ROS Nodes* running on the network, some of which we will interact with, but others may just be running in the background. It is often useful to know exactly what *is* running on the ROS network, and there are a number of ways to do this.

#### :material-pen: Exercise 4: Visualising the ROS Network {#exNet}

1. There shouldn't be anything running in Terminals **1** or **2** now, so return to **TERMINAL 1** and use the `rosnode` command to *list* the nodes that are currently running on the robot:

    ***
    **TERMINAL 1:**
    ```bash
    rosnode list
    ```
    ***

    You should see a list of at least 7 items.

2. We can visualise the connections between the active nodes by using a ROS node called `rqt_graph`. Launch this as follows:

    ***
    **TERMINAL 1:**
    ```bash
    rosrun rqt_graph rqt_graph
    ```
    ***
    
3. In the window that opens, select `Nodes/Topics (active)` from the dropdown menu in the top left. 

    What you should then see is a map of all the nodes in the list from above (as ovals), and arrows to illustrate the flow of information between them. This is a visual representation of the ROS network!
    
    Items that have a rectangular border are *ROS Topics*. ROS Topics are essentially communication channels, and ROS nodes can read (*subscribe*) or write (*publish*) to these topics to access sensor data, pass information around the network and make things happen.

A ROS Robot could have hundreds of individual nodes running simultaneously to carry out all its necessary operations and actions. Each node runs independently, but uses *ROS communication methods* to communicate and share data with the other nodes on the ROS Network.

## Publishers and Subscribers: A *ROS Communication Method* 

ROS Topics are key to making things happen on a robot. Nodes can publish (*write*) and/or subscribe to (*read*) ROS Topics in order to share data around the ROS network. Data is published to topics using *ROS Messages*. We were actually publishing messages to a topic when we made the robot move using the Teleop node in the previous exercises.

Let's have a look at this in a bit more detail...

#### :material-pen: Exercise 5: Exploring ROS Topics and Messages {#exTopicMsg}

Much like the `rosnode list` command, we can use `rostopic list` to list all the *topics* that are currently active on the ROS network.

1. Close down the `rqt_graph` window if you haven't done so already. This will release **TERMINAL 1** so that we can enter commands in it again. Return to this terminal window and enter the following:

    ***
    **TERMINAL 1:**
    ```bash
    rostopic list
    ```
    ***

    A much larger list of items should be printed to the terminal now. See if you can spot the `/cmd_vel` item in the list.
    
    This topic is used to control the velocity of the robot (*'command velocity'*).

1. Let's find out more about this using the `rostopic info` command.

    ***
    **TERMINAL 1:**
    ```bash
    rostopic info /cmd_vel
    ```
    ***

    This should provide an output similar to the following: 
    
    ``` { .txt .no-copy }
    Type: geometry_msgs/Twist

    Publishers: None

    Subscribers:
     * /turtlebot3_core (http://dia-waffleX:#####/)
    ```

    This tells us a few things: <a name="rostopic_info_explained"></a>
    
    1. The `/cmd_vel` topic currently has no publishers (i.e. no other nodes are currently writing data to this topic).
    1. The `/turtlebot3_core` node is subscribing to the topic. The `/turtlebot3_core` node turns motor commands into actual wheel motion, so it monitors the topic (i.e. *subscribes* to it) to see when a velocity command is published to it.
    1. The *type* of message used by the `/cmd_vel` topic is called: `geometry_msgs/Twist`. 
    
        The message type has two parts: `geometry_msgs` and `Twist`. `geometry_msgs` is the name of the ROS package that this message belongs to and `Twist` is the actual message *type*. 

        We have just learnt then, that if we want to make the robot move we need to publish `Twist` messages to the `/cmd_vel` topic. 

1. We can use the `rosmsg` command to find out more about the `Twist` message:

    ***
    **TERMINAL 1:**
    ```bash
    rosmsg info geometry_msgs/Twist
    ```
    ***

    From this, we should obtain the following:

    ``` { .txt .no-copy }
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
    ```

    Let's find out what it all means...

## Velocity Control

The motion of any mobile robot can be defined in terms of its three *principal axes*: `X`, `Y` and `Z`. In the context of our TurtleBot3 Waffle, these axes (and the motion about them) are defined as follows:

<figure markdown>
  <!-- ![](../../images/waffle/principal_axes.svg){width=600} -->
  ![](../images/waffle/principal_axes.svg){width=600}
</figure>

In theory then, a robot can move *linearly* or *angularly* about any of these three axes, as shown by the arrows in the figure. That's six *Degrees of Freedom* (DOFs) in total, achieved based on a robot's design and the actuators it is equipped with. Take a look back at the `rosmsg info` output in **TERMINAL 1**. Hopefully it's a bit clearer now that these topic messages are formatted to give a ROS Programmer the ability to *ask* a robot to move in any one of its six DOFs. 

``` { .txt .no-copy }
geometry_msgs/Vector3 linear
  float64 x  <-- Forwards (or Backwards)
  float64 y  <-- Left (or Right)
  float64 z  <-- Up (or Down)
geometry_msgs/Vector3 angular
  float64 x  <-- "Roll"
  float64 y  <-- "Pitch"
  float64 z  <-- "Yaw"
```

Our TurtleBot3 robot only has two motors, so it doesn't actually have six DOFs! The two motors can be controlled independently, which gives it what is called a *"differential drive"* configuration, but this still only allows it to move with **two degrees of freedom** in total, as illustrated below.

<figure markdown>
  <!-- ![](../../images/waffle/velocities.svg?width=20cm) -->
  ![](../images/waffle/velocities.svg){width=600}
</figure>

It can therefore only move **linearly** in the **x-axis** (*Forwards/Backwards*) and **angularly** in the **z-axis** (*Yaw*). 

#### :material-pen: Exercise 6: Creating Your First Python Node {#exSimpleVelCtrl}

Making a robot move with ROS is simply a case of publishing the right ROS Message (`Twist`) to the right ROS Topic (`/cmd_vel`). In some of the previous exercises above you used the Keyboard Teleop node to drive the robot around, a bit like a remote control car. In the background here all that was really happening was that the Teleop node was converting our keyboard button presses into velocity commands and publishing these to the `/cmd_vel` topic.

In reality, robots need to be able to navigate complex environments autonomously, which is quite a difficult task, and requires us to build bespoke applications. We can build these applications using Python, and we'll look at the core concepts behind this now by building a simple node that will allow us to make our robot a bit more "autonomous". What we will do here forms the basis of the more complex applications that you will learn about in [Assignment #1](../../com2009/assignment1/) and implement in [Assignment #2](../../com2009/assignment2/) to bring a real robot to life!

1. You will create your first ROS node inside your team's `com2009_team999` ROS package, which you should have cloned to the laptop earlier on. This package should now correctly reside within the Catkin Workspace on the laptop's filesystem. Navigate to this from **TERMINAL 1** using the `roscd` command:

    ***
    **TERMINAL 1:**
    ``` { .bash .no-copy }
    roscd com2009_team999/ 
    ```
    Replacing `com2009_team999` accordingly.
    ***

1. Then, use the `cd` command to move into the `src` directory that should already exist within your package:

    ***
    **TERMINAL 1:**
    ```bash
    cd src/ 
    ```
    ***

1. In here, create a Python file called `simple_move_square.py` using the `touch` command:

    ***
    **TERMINAL 1:**
    ```bash
    touch simple_move_square.py
    ```
    ***

1. You'll need to change the *execution permissions* for this file in order to be able to run it later on. You'll learn more about this in Assignment #1 but, for now, simply run the following command:

    ***
    **TERMINAL 1:**
    ```bash
    chmod +x simple_move_square.py
    ```
    ***

2. Now we want to edit this file, and we'll do that using *Visual Studio Code* (VS Code):

    ***
    **TERMINAL 1:** 
    ```bash
    code .
    ```
    ***

    !!! note
        Don't forget to include the `.` at the end there, it's important!!
    
3. Once VS Code launches, open up the `simple_move_square.py` file, which should be visible in the file explorer on the left-hand side of the VS Code window. Paste the following content into it:

    ```py title="simple_move_square.py"
    #!/usr/bin/env python3

    --8<-- "snippets/move_square_timed.py"
    ```

    1. `rospy` is the ROS client library for Python. We need this so that our Python node can interact with ROS.
    2. We know from earlier that in order to make a robot move we need to publish messages to the `/cmd_vel` topic, and that this topic uses `Twist` messages from the `geometry_msgs` package. This is how we import that message, from that package, in order to create velocity commands in Python (which we'll get to shortly...)
    3. Before we do anything we need to initialise our node to register it on the ROS network with a name. We're calling it "move_waffle" in this case, and we're using `anonymous=True` to ensure that there are no other nodes of the same name already registered on the network.
    4. We want our main `while` loop (when we get to that bit) to execute 10 times per second (10 Hz), so we create a `rate` object here which will be used to control the rate of the main loop later...
    5. Here we are setting up a publisher to the `/cmd_vel` topic so that the node can write `Twist` messages to make the robot move.
    6. We're instantiating a `Twist` message here and calling it `vel` (we'll assign velocity values to this in the `while` loop later on). A `Twist` message contains six different components that we can assign values to. Any idea [what these six values might represent](#velocity-control)?  
    7. What time is it right now? (This will be useful to compare against in the while loop.)
    8. We're entering the main `while` loop now. This `rospy.is_shutdown()` function will read `False` unless we request for the node to be stopped (by pressing ++ctrl+c++ in the terminal). Once it turns `True` the `while` loop stops.
    9. Here we're comparing the time now to the time the last time we checked, to tell us how much time has elapsed (in seconds) since then. We'll use that information to decide what to do...  
    10. The "transition" state is used to stop the robot (if necessary), and check the time again.
    11. In "state1" we set velocities that will make the robot move forwards (linear-X velocity only). If the elapsed time is greater than **2 seconds** however, we move on to "state2".
    12. In "state2" we set velocities that will make the robot turn on the spot (angular-Z velocity only). In this case, if the elapsed time is greater than **4 seconds**, we move back to "state1".
    13. Regardless of what happens in the `if` statements above, we always publish a velocity command to the `/cmd_vel` topic here (i.e. every loop iteration).
    14. We created a `rate` object earlier, and we use this now to make sure that each iteration of this `while` loop takes exactly the right amount of time to maintain the rate of execution that we specified earlier (10 Hz).
    15. Here we're importing some mathematical operators that might be useful... 

        | Mathematical Operation | Python Implementation |
        | :---: | :---: |
        | $\sqrt{a+b}$ | `#!python sqrt(a+b)` |
        | $a^{2}+(bc)^{3}$ | `#!python pow(a, 2) + pow(b*c, 3)` |
        | $\pi r^2$ | `#!python pi * pow(r, 2)` |

    Click on the :material-plus-circle: icons above to expand the code annotations. Read these carefully to ensure that you understand what's going on and how this code works.

4. Now, go back to **TERMINAL 1** and run the code.

    !!! note
        Make sure the robot is on the floor and has enough room to roam around before you do this!
    
    ***
    **TERMINAL 1:**
    ``` { .bash .no-copy }
    rosrun com2009_team999 simple_move_square.py
    ```
    ***
    
    Observe what the robot does. When you've seen enough, enter ++ctrl+c++ in **TERMINAL 1** to stop the node from running, which should also stop the robot from moving.
    
5. As the name may suggest, the aim here is to make the robot follow a square motion path. What you may have observed when you actually ran the code is that the robot doesn't actually do that! We're using a time-based approach to make the robot switch between two different states continuously:
    1. Moving forwards
    2. Turning on the spot
    
    Have a look at the code to work out how much time the robot will currently spend in each state.
    
6.  The aim here is to make the robot follow a **0.5m x 0.5m square** motion path.  In order to properly achieve this you'll need to adjust the timings, or the robot's velocity, or both. Edit the code so that the robot actually follows a **0.5m x 0.5m square motion path**!

## SLAM

Simultaneous Localisation and Mapping (SLAM) is a sophisticated tool that is built into ROS. Using data from the robot's LiDAR sensor, plus knowledge of how far the robot has moved[^odom] the robot is able to create a map of its environment *and* keep track of its location within that environment at the same time. IN the exercise that follows you'll see easy it is to implement SLAM on the real robot.  

[^odom]: You'll learn much more about "Robot Odometry" in [Assignment #1 Part 2](../../com2009/assignment1/part2), and in the COM2009 Lectures.

#### :material-pen: Exercise 7: Using SLAM to create a map of the environment {#exSlam}

1. In **TERMINAL 1** enter the following command to launch all the necessary SLAM nodes on the laptop:

    ***
    **TERMINAL 1:**
    ```bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch
    ```
    
    ??? tip "Robotics Laptop Tip"
        This command is also available as an alias: `tb3_slam`!

    ***

    This will launch RViz once again, where you should now be able to see a model of the Waffle from a top-down view surrounded by green dots representing the real-time LiDAR data. The SLAM tools will already have begun processing this data to start building a map of the boundaries that are currently visible to the Waffle based on its location in the environment.

    !!! note
        To begin with your robot may just appear as a white shadow (similar to the left-hand image below). It may take some time for the robot to render correctly (like the right-hand image) as the SLAM processes and data communications catch up with one another. 
        
        <figure markdown>
          ![](../images/waffle/slam.png){width=600px}
        </figure>
        
        This can sometimes take up to a minute or so, so please be patient! If (after a minute) nothing has happened, then speak to a member of the teaching team.

1. Return to **TERMINAL 2** and launch the `turtlebot3_teleop_keyboard` node again. Start to drive the robot around *slowly* and *carefully* to build up a complete map of the area.
    
    !!! tip
        It's best to do this slowly and perform multiple circuits of the whole area to build up a more accurate map.

1. Once you're happy that your robot has built up a good map of its environment, you can save this map using a node called `map_saver` from a package called `map_server`:

    1. First, create a new directory within your team's ROS package on the laptop. We'll use this to save maps in. Open up a new terminal instance (**TERMINAL 3**) and navigate to the root of your team's ROS package with `roscd` again:

        ***
        **TERMINAL 3:**
        ``` { .bash .no-copy }
        roscd com2009_team999
        ```
        ***

    1. Create a directory in here called `maps`: 
        
        ***
        **TERMINAL 3:**
        ```bash
        mkdir maps/
        ```
        ***

    2. Navigate into this directory:

        ***
        **TERMINAL 3:**
        ```bash
        cd maps/
        ```
        ***

    2. Then, use `rosrun` to run the `map_saver` node from the `map_server` package to save a copy of your map:

        ***
        **TERMINAL 3:**
        ``` { .bash .no-copy }
        rosrun map_server map_saver -f {map_name}
        ```
        
        Replacing `{map_name}` with an appropriate name for your map. This will create two files: 
        
        1. a `{map_name}.pgm` 
        2. a `{map_name}.yaml` file
        
        ...both of which contain data related to the map that you have just created.

        ***

    1. The `.pgm` file can be opened using an application called `eog` on the laptop: 
    
        ***
        **TERMINAL 3:**
        ``` { .bash .no-copy }
        eog {map_name}.pgm
        ```
        ***

1. Return to **TERMINAL 1** and close down SLAM by pressing ++ctrl+c++. The process should stop and RViz should close down.

1. Close down the Keyboard Teleop node in **TERMINAL 2** as well if that's still running.

## Wrapping Up

#### :material-pen: Exercise 8: Pushing Changes to Your ROS Package Back to GitHub {#exGitPush}

Having completed the above exercises your team's ROS package should now contain a new ROS Node called `simple_move_square.py`, a new directory called `maps`, and a couple of map files within this. You can commit these changes to your repo now using Git, and then push these to GitHub. While the files themselves aren't particularly important for Assignment #2, this will at least illustrate the process for pushing changes from the laptop in future.

!!! warning "WiFi"
    Once again, make sure the laptop is now connected to "eduroam" in order to be able to access the internet.

1. Head back to **TERMINAL 3** and make sure that you are located in the root of your team's package:

    ***
    **TERMINAL 3:**
    ``` { .bash .no-copy }
    roscd com2009_team999/
    ```
    ***

1. Check the status of your Git repo to identify the changes that have been made:

    ***
    **TERMINAL 3:**
    ```bash
    git status
    ```
    ***

1. Stage all the changes that have been made:

    ***
    **TERMINAL 3:**
    ```bash
    git add .
    ```
    ***

1. Then commit them:

    ***
    **TERMINAL 3:**
    ```bash
    git commit -m "Getting started in the lab with the Waffles"
    ```
    ***

1. Finally, push these to GitHub:

    ***
    **TERMINAL 3:**
    ```bash
    git push origin main
    ```
    ***

<center>
  <strong> 
    <em>
      Getting Started Exercises complete!
    </em>
  </strong>
</center>

Continue onto the next page now for the shutdown procedures that you need to follow at the end of each lab session...
