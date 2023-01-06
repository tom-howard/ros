---
title: Week 1 ROS & Linux Basics 
subtitle: In this first week you will learn the basics of ROS and become familiar with some key tools and principles of this framework, allowing you to program robots and work with ROS applications effectively. 
---

<!-- {{% textalign center %}}
*You should be able to complete the exercises on this page within a two-hour lab session*.
{{% /textalign %}} -->

## Introduction

### Aims

In this first week you will learn the basics of ROS and become familiar with some key tools and principles of this framework, which will allow you to program robots and work with ROS applications effectively.  For the most part, you will interact with ROS using the *Linux command line* and so you will also become familiar with some key Linux command line tools that will help you.  Finally, you will learn how to create some basic ROS Nodes using Python and get a taste of how ROS topics and messages work.

### Intended Learning Outcomes

By the end of this session you will be able to:  

1. Control a TurtleBot3 Robot, in simulation, using ROS.
1. Launch ROS applications using `roslaunch` and `rosrun`.
1. Interrogate running ROS applications using key ROS command line tools.
1. Create a ROS package comprised of multiple nodes and program these nodes (in Python) to communicate with one another using ROS Communication Methods.
1. Navigate a Linux filesystem and learn how to do various filesystem operations from within a Linux Terminal.

### Quick Links

* [Exercise 1: Launching a simulation and making a ROS robot move](#ex1)
* [Exercise 2: Exploring a ROS Package](#ex2)
* [Exercise 3: Visualising the ROS Network](#ex3)
* [Exercise 4: Exploring ROS Topics and Messages](#ex4)
* [Exercise 5: Creating your own ROS Package](#ex5)
* [Exercise 6: Creating a publisher node](#ex6)
* [Exercise 7: Creating a subscriber node](#ex7)
* [Exercise 8: Creating a launch file](#ex8)

### Additional Resources

* [The Publisher Code (for Exercise 6)](publisher)
* [The Subscriber Code (for Exercise 7)](subscriber)

## First Steps

#### Exercise 1: Launching a simulation and making a ROS robot move {#ex1}

1. If you haven't done so already, launch your WSL-ROS environment by running the WSL-ROS shortcut in the Windows Start Menu hello ([see here for detailed instructions](/wsl-ros/getting-started)). This should open up a *terminal application* and an *Ubuntu terminal instance*.  We'll refer to this terminal instance as **TERMINAL 1**.
1. In the terminal enter the following command to launch a simulation of a TurtleBot3 Waffle in an empty world:  
        
    ***
    **TERMINAL 1:** 
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```    
    ***

1. A Gazebo simulation window should open and within this you should see a TurtleBot3 Waffle (similar to [our real robots](/about/robots) that you'll work with later):

    <figure markdown>
      ![](/images/gazebo/tb3_empty_world.png){width="800"}
    </figure>

1. With your Gazebo Simulation up and running, return to the terminal application and open up a new Ubuntu terminal instance (**TERMINAL 2**) by pressing the *New Tab* button: 
    
    <figure markdown>
      ![](/images/wsl/wt_new_tab.svg)
    </figure>

    (or, alternatively, press the `Ctrl+Shift+T` keyboard shortcut).
    
1. In the new terminal instance enter the following command:<a name="teleop"></a>

    ***
    **TERMINAL 2:**
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
    ***

1. Follow the instructions provided in the terminal to drive the robot around in its simulated environment.

##### Summary

You have so far launched two separate ROS applications using the `roslaunch` command. `roslaunch` is one way to launch ROS programs. As you should have observed from the above examples, we use this command in the following way:

```bash
roslaunch {package name} {launch file}
```

The command takes two parameters as inputs: `{package name}` is the name of the *ROS package* that contains the functionality that we want to execute and `{launch file}` is a file within that package that tells ROS exactly what functionality within the package we want to launch.

## ROS Packages

ROS applications are organised into *packages*. Packages are basically folders containing scripts, configurations and launch files (ways to launch those scripts and configurations), all of which relate to some common robot functionality. ROS uses packages as a way to organise all the programs running on a robot. 

!!! info  
    The package system is a fundamental concept in ROS and all ROS programs are organised in this way.

#### Exercise 2: Exploring a ROS Package {#ex2}

`roscd` is a **ROS command** that allows us to navigate to the directory of any ROS package installed on our system, without us needing to know the path to the package beforehand.

1. Open up a new terminal instance (**TERMINAL 3**) and use the `roscd` command to navigate to the `turtlebot3_teleop` package directory on the Linux filesystem:
  
    ***
    **TERMINAL 3:**
    ```bash
    roscd turtlebot3_teleop
    ```
    ***

    The terminal prompt should have changed to illustrate where on the filesystem the `roscd` command has just taken you:
    
    <figure markdown>
      ![](/images/ros-cli/tb3_teleop_dir.svg)
    </figure>

1. `pwd` is a **Linux command** which tells us the current filesystem location of our terminal.  Enter this command to confirm what the terminal prompt has told us.
    
    So, now we know *where* the `turtlebot3_teleop` package is located on our machine, and we can then use more Linux commands to explore this further:

1. `ls` is a **Linux command** which *lists* the contents of the current directory.  Use this to list the contents of the `turtlebot3_teleop` package directory.

1. `ls` on its own will simply list the items in the current directory, try this first.

1. Then, use the `-F` option to find out a little more:
    
    ***
    **TERMINAL 3:**
    ```bash
    ls -F
    ```
    ***
    
    You will notice that the output has now changed slightly: items followed by a `/` are folders (aka *"directories"*) and items without the `/` are files (files will often have a file extension too).
    
    !!! question "Questions" 
        1. How many items were there in the `turtlebot3_teleop` package directory?
        1. How many of these were directories and how many were files? 

    *Launch files* for a package are typically located in a *launch* folder within the package directory. *Did you notice a `launch` folder in the output of the `ls` command above?*

1. `cd` is a **Linux command** that allows us to **C**hange the **D**irectory that the terminal is currently located in. Use this to navigate into the `turtlebot3_teleop` package `launch` folder and then use `ls` again to see what's in there. 

    In this folder you should see the `turtlebot3_teleop_key.launch` file that we executed with the `roslaunch` command in [Exercise 1](#ex1).  We will now have a look at the contents of this file...

1. `cat` is a **Linux command** that we can use to display the contents of a file in the terminal.  Use this to display the contents of the `turtlebot3_teleop_key.launch` file.

    ***
    **TERMINAL 3:**
    ```bash
    cat turtlebot3_teleop_key.launch
    ```
    ***

##### Summary {#package_attributes}

From the output of `cat` in the step above you should have noticed that the contents of a launch file are contained within a `<launch>` tag:

```xml
<launch>
  ... 
</launch>
```

Within that, we also have (amongst other things) a `<node>` tag which tells ROS exactly what scripts (*"executables"*) to launch *and* how to launch them:

```xml    
<node pkg="turtlebot3_teleop" type="turtlebot3_teleop_key" name="turtlebot3_teleop_keyboard" output="screen">
</node>
```

The attributes here have the following meaning:
* `pkg`: The name of the *ROS package* containing the functionality that we want to launch.
* `type`: The full name of the script (i.e. *ROS Node*) that we want to execute within that package (including the file extension, if it has one).
* `name`: A descriptive name that we want to give to the ROS node, which will be used to register it on the ROS Network.
* `output`: The place where any output from the node will be printed (either *screen* where the output will be printed to our terminal window, or *log* where the output will be printed to a log file).

## ROS Nodes {#nodes}

ROS *Nodes* are executable programs that perform specific robot tasks and operations, such as remote (or "teleoperational") control, as we saw in the earlier example. 

The packages that we will create throughout this course will contain nodes, launch files and other things too, and we'll explore this a little more shortly.

The `turtlebot3_teleop` package that we have just interrogated here however is fairly minimal and only contains launch files... the nodes are actually located elsewhere, in a directory called `/opt/ros/noetic/lib/turtlebot3_teleop/` (this is just the way things are organised for pre-installed packages).

!!! question "Questions"  
    1. What is the name of the node that is launched by the `turtlebot3_teleop_key.launch` file?
    1. See if you can find this in `/opt/ros/noetic/lib/turtlebot3_teleop/`.
    1. Use the Linux/ROS command line tools that you have learnt about so far to interrogate the file...

A ROS Robot might have hundreds of individual nodes running simultaneously to carry out all its necessary operations and actions. Each node runs independently, but uses *ROS communication methods* to communicate and share data with the other nodes on the ROS Network.

## The ROS Network

You can use the `rosnode` **ROS command** to view all the nodes that are currently active on a ROS Network.

#### Exercise 3: Visualising the ROS Network {#ex3}

You should currently have three terminal windows active: the first in which you launched the Gazebo simulation (**TERMINAL 1**), the second with your `turtlebot3_teleop_key` node active (**TERMINAL 2**), and the third where you explored the contents of the `turtlebot3_teleop` package directory (**TERMINAL 3**).  **TERMINAL 3** should now be idle.

1. In **TERMINAL 3** enter `cd ~` to go back to your home directory ([remember that `~` is an alias for your home directory](/wsl-ros/getting-started/#linux-term)).
1. Use the following command to have a look at which nodes are currently active on the network:

    ***
    **TERMINAL 3:**
    ```bash
    rosnode list
    ```
    ***

    Only a handful of nodes should be listed:

    ```bash
    /gazebo
    /gazebo_gui
    /rosout
    /turtlebot3_teleop_keyboard
    ```

1. We can visualise the connections between the active nodes by using the `rqt_graph` node within the `rqt_graph` package. We can use `rosrun` to launch this node directly (you might get some error messages, but don't worry about them):

    ***
    **TERMINAL 3:**
    ```bash
    rosrun rqt_graph rqt_graph
    ```
    ***

    A new window should then open, displaying something similar to the following (hover over the diagram to enable colour highlighting):

    ![A visualisation of all the ROS nodes active on the system and the flow of information between them](/images/rqt/graph.png)

    This tool shows us that the `/turtlebot3_teleop_keyboard` and `/gazebo` nodes are communicating with one another. The direction of the arrow tells us that the `/turtlebot3_teleop_keyboard` node is a *Publisher* and the `/gazebo` node is a *Subscriber*. The two nodes communicate via a **ROS Topic**, in this case the `/cmd_vel` topic, and on this topic the `/turtlebot3_teleop_keyboard` node publishes **messages**.

## Publishers and Subscribers: A *ROS Communication Method* 

ROS Topics are key to making things happen on a robot. Nodes can publish (*write*) and/or subscribe to (*read*) ROS Topics in order to share data around the ROS network. Data is published to topics using *ROS Messages*. We were actually publishing messages to a topic when we made the robot move using the Teleop node in the previous exercises.

Let's have a look at this in a bit more detail...

#### Exercise 4: Exploring ROS Topics and Messages {#ex4}

We can find out more about the `/cmd_vel` topic by using the `rostopic` *ROS command*.

1. In a new terminal instance (**TERMINAL 4**) type the following:

    ***
    **TERMINAL 4:**
    ```bash
    rostopic info /cmd_vel
    ```
    ***

    This should provide an output similar to the following:
    
    ```bash
    Type: geometry_msgs/Twist

    Publishers:
        * /turtlebot3_teleop_keyboard (http://localhost:#####/)

    Subscribers:
        * /gazebo (http://localhost:#####/)
    ```

    This confirms what we discovered earlier about the publisher(s) and subscriber(s) to the `/cmd_vel` topic.  In addition, this also tells us the topic *type*, or the *type of message* that is being published on this topic.

    This tells us a few things:
    
    1. The `/turtlebot3_teleop_keyboard` node is currently publishing (i.e. writing data) to the `/cmd_vel` topic, confirming what we saw from the `rqt_graph` node before.
    1. The `/gazebo` node is subscribing to the topic. This node is the Gazebo application that's running the simulation of the robot. The node therefore monitors (i.e. *subscribes* to) the `/cmd_vel` topic and makes the robot move in the simulator whenever a velocity command is published.
    1. The *type* of message used by the `/cmd_vel` topic is called: `geometry_msgs/Twist`. 
    
        The message type has two parts: `geometry_msgs` and `Twist`. `geometry_msgs` is the name of the ROS package that this message belongs to and `Twist` is the actual message *type*. 

        We have just learnt then, that if we want to make the robot move we need to publish `Twist` messages to the `/cmd_vel` topic.

1. We can use the `rosmsg` **ROS command** to provide further information about this message, or any other message that we may be interested in:

    ***
    **TERMINAL 4:**
    ```bash
    rosmsg info geometry_msgs/Twist
    ```
    ***

    From this, we should obtain the following:

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

    We'll learn more about what this means next week.

1. To finish, shut down any active terminal processes by entering `Ctrl+C` in any that still have processes running (Terminals **1**, **2** and **3**). The associated Gazebo and rqt_graph windows should close as a result of this too.

#### Exercise 5: Creating your own ROS Package {#ex5}

In a minute or two you will create some simple publisher and subscriber nodes in Python and send messages between them. As we learnt earlier though, ROS applications must be created within packages, and so we need to create a package first in order to start creating our own ROS nodes. 

ROS provides a tool to create a new ROS package and ensure that all the essential elements are present: `catkin_create_pkg`.

It's important to work in a specific filesystem location when we create and work on our own ROS packages, so that ROS can access and build everything appropriately. These spaces are called *"Catkin Workspaces"* and one has already been created in the WSL-ROS environment, called `catkin_ws`[^why_catkin]:

```bash
/home/student/catkin_ws/
```
Or:
```bash
~/catkin_ws/
```

[^why_catkin]: "The name catkin comes from the tail-shaped flower cluster found on willow trees -- a reference to Willow Garage where catkin was created." (According to [ROS.org](http://wiki.ros.org/catkin/conceptual_overview#Overview)) 

1. Navigate to the `catkin_ws` folder by using the Linux `cd` command. In **TERMINAL 1** enter the following:

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/catkin_ws/
    ```
    ***

1. Inside the catkin workspace there is a directory called `src` (use the `ls` command to confirm this). All new packages need to be located in the `src` folder, so we need to be here when we use the `catkin_create_pkg` tool to create a new package. So, use the `cd` command again to navigate to the `catkin_ws/src` folder:

    ***
    **TERMINAL 1:**
    ```bash
    cd src
    ```
    ***

1. Now, use the `catkin_create_pkg` script to create a new package called `week1_pubsub`, and define `std_msgs` and `rospy` as dependencies:

    ***
    **TERMINAL 1:**
    ```bash
    catkin_create_pkg week1_pubsub std_msgs rospy
    ```
    ***

    !!! question
        What did the `catkin_create_pkg` tool just do? (**Hint**: there were *four* things, and it will have told you about them!)
    
1. Navigate into this new package directory and use `ls` to list the content that has been created by the `catkin_create_pkg` tool.

    Catkin packages are typically organised in the following way, and have a few essential features that **must** be present in order for the package to be valid:

        package_folder/    -- All packages must be self-contained within their own root folder [essential]
        |-launch/          -- A folder for launch files (optional)
        |-src/             -- A folder for source files (python scripts etc)
        |-CMakeLists.txt   -- Rules for compiling the package [essential]
        `-package.xml      -- Information about the package [essential]

    You will have noticed that the `catkin_create_pkg` tool made sure that the essential features of a Catkin Package were created when we asked it to build the `week1_pubsub` package above.

1. Before we do anything else, it's good practice to now run `CMake` on the package (using `catkin build`) to register it on our ROS system and make sure there are no errors with its definition so far:

    ***
    **TERMINAL 1:**
    ```bash
    catkin build week1_pubsub
    ```
    Finally, "re-source" your environment[^source-bashrc] using the following command:
    ```bash
    source ~/.bashrc
    ```
    ***

    ... and you're good to go.

    !!! warning
        You will need run `#!bash source ~/.bashrc` in any other terminals that you have open too, in order for the changes to propagate through to these as well!
    
[^source-bashrc]: What does `source ~/.bashrc` do? [See here for an explanation](https://devconnected.com/source-command-on-linux-explained/#Source_to_update_your_current_shell_environment_bashrc).

#### Exercise 6: Creating a publisher node {#ex6}

1. Within your `week1_pubsub` package directory, navigate to the `src` folder using the `cd` command.
1. `touch` is a **Linux command** that we can use to create an empty file. Use this to create an empty file called `publisher.py`, which we will add content to shortly:

    ***
    **TERMINAL 1:**
    ```bash
    touch publisher.py
    ```
    ***

1. Use `ls` to verify that the file has been created, but use the `-l` option with this, so that the command provides its output in *"a long listing format"*:

    ***
    **TERMINAL 1:**
    ```bash
    ls -l
    ```
    ***

    This should output something similar to the following:

    ```txt
    total 0
    -rw-r--r-- 1 student student 0 MMM DD HH:MM publisher.py
    ```

    This confirms that the file exists, and the `0` in the middle of the bottom line there indicates that the file is empty (i.e. its current size is 0 bytes), which is what we'd expect.

1. We therefore now need to open the file and add content to it. As discussed on [the Getting Started page](/wsl-ros/getting-started/#vscode), we'll be using Visual Studio Code as our IDE for this work. It's important to launch this in a very specific way in order for it to work properly with the WSL-ROS environment, [so follow the instructions here to get this up and running now](/wsl-ros/vscode)!

1. [Make sure that the "Remote - WSL" VS Code extension is enabled within the WSL-ROS environment](/wsl-ros/vscode/#verify)!!

1. Using the VS Code File Explorer, navigate to your `week1_pubsub` package directory (`~/catkin_ws/src/week1_pubsub/`), locate the `publisher.py` file that you have just created in the `/week1_pubsub/src/` folder and click on the file to open it. 

1. Once opened, copy [the code provided here](publisher) into the empty file and save it. <a name="ex6_ret"></a>
    
    !!! note
        It's important that you understand how this code works, so make sure that you **read the annotations**!
    
1. We can now run this node using the `rosrun` **ROS command**. However, because we closed everything down earlier on, the *ROS Master* is no longer active. First then, we need to re-launch it manually using `roscore`:

    ***
    **TERMINAL 1:**
    ```bash
    roscore
    ```
    ***

1. Then, in **TERMINAL 2**, use `rosrun` to execute the `publisher.py` script that you have just created by providing the relevant information to the `rosrun` command as follows: `rosrun {package name} {script name}`, i.e.:

    ***
    **TERMINAL 2:**
    ```bash
    rosrun week1_pubsub publisher.py
    ```
    ***

    ... Hmmm, something not quite right? If you typed the command exactly as above and then tried to run it, you probably just received the following error:

    ```txt
    [rosrun] Couldn't find executable named publisher.py below /home/student/catkin_ws/src/week1_pubsub
    [rosrun] Found the following, but they're either not files,
    [rosrun] or not executable:
    [rosrun]   /home/student/catkin_ws/src/week1_pubsub/src/publisher.py
    ``` 

    The clue there is the word *"executable"*. When we create a file, using `touch` it is given certain *permissions*. Run `ls -l` again (making sure that your terminal is in the right location: `~/catkin_ws/src/week1_pubsub/src/`).
        
    The first bit tells us about the permissions that are currently set: `-rw-r--r--`. This tells us *who* has permission to do *what* with this file and (currently) the first bit: `-rw-`, tells us that we (as the user `student`) have permission to **R**ead or **W**rite to it. There is a *third* option we can set too though, which is the *execute* permission, and we can set this using the `chmod` **Linux command**...

1. Run the `chmod` command as follows:

    ***
    **TERMINAL 2:**
    ```bash
    chmod +x publisher.py
    ```
    ***

1. Now, run `ls -l` again to see what has changed:
    
    ***
    **TERMINAL 2:**
    ```bash
    ls -l
    ```
    ***

    We have now granted permission for the file to be e**X**ecuted too:
    
    ```txt
    -rwxr-xr-x 1 student student 1557 MMM DD HH:MM publisher.py
    ```

1. OK, now use `rosrun` again to (*hopefully!*) run the `publisher.py` node (remember: `rosrun {package name} {script name}`).
    
    If you see a message in the terminal similar to the following then the node has been launched successfully:
        
    ```txt
    [INFO] [#####]: The 'simple_publisher' node is active...
    ```

    Phew!

1. We can further verify that our publisher node is running using a number of different tools. Try running the following commands in **TERMINAL 3**:

    1. `rosnode list`: This will provide a list of all the nodes that are currently active on the system. Verify that the name of our publisher node is visible in this list.
    1. `rostopic list`: This will provide a list of the topics that are currently being used by nodes on the system. Verify that the name of the topic that our publisher is publishing messages to is present within this list.

### Using the `rostopic` command {#rostopic}

So far we have used the `rostopic` ROS command with two additional arguments:

* `list`: to provide us with a *list* of all the topics that are active on our ROS system, and
* `info`: to provide us with *information* on a particular topic of interest.

We can use the *autocomplete functionality* of the Linux terminal to provide us with a list of *all* the available options that we can use with the `rostopic` command.  To do this you can type `rostopic` followed by a `Space` and then press the `Tab` key twice:

```bash
rostopic[SPACE][TAB][TAB]
```

You should then be presented with a list of the available arguments for the `rostopic` command:

<!-- TODO: a gif instead of static image! -->

<figure markdown>
  ![](/images/ros-cli/rostopic_autocomplete.png)
</figure>

* `rostopic hz {topic name}` provides information on the frequency (in Hz) at which messages are being published to a topic:

    ```bash
    rostopic hz /chatter
    ```

    This should tell us that our publisher node is publishing messages to the `/chatter` topic at (or close to) 10 Hz, which is exactly what we ask for in the `publisher.py` file (in the `__init__` part of our `Publisher` class). Press `Ctrl+C` to stop this command.

* `rostopic echo {topic name}` shows the messages being published to a topic:

    ```bash
    rostopic echo /chatter
    ```

    This will provide a live stream of the messages that our `publisher.py` node is publishing to the `/chatter` topic. Press `Ctrl+C` to stop this.

* We can see some additional options for this command by viewing the help documentation:

    ```bash
    rostopic echo -h
    ```

    From here, for instance, we can learn that if we just wanted the echo command to display a set number of messages from the `/chatter` topic we could use the `-n` option. To display the most recent two messages only, for example:

    ```bash
    rostopic echo /chatter -n2
    ```

#### Exercise 7: Creating a subscriber node {#ex7}

You will now create another node to *subscribe* to the topic that our publisher node is broadcasting messages to, to illustrate how information can be passed from one node to another, via topic messages.

1. In **TERMINAL 3** use the filesystem commands that were introduced earlier (`cd`, `ls` and `roscd`) to navigate to the `src` folder of your `week1_pubsub` package.
1. Use the same procedure as before to create a new empty Python file called `subscriber.py` and remember to make it executable! <a name="ex7_ret"></a>
1. Then, open the newly created `subscriber.py` file in VS Code, paste in [the code here](subscriber) and save it. Once again, it's important that you understand how this code works, so **make sure you read the code annotations**! 

1. Use `rosrun` to execute your newly created `subscriber.py` node (remember: `rosrun {package name} {script name}`). If your publisher and subscriber nodes are working correctly you should see an output like this:
    
    <figure markdown>
      ![](images/subscriber_output.gif)
    </figure>

1. As before, we can find out what nodes are running on our system by using the `rosnode list` command. Open a new terminal window (**TERMINAL 4**), run this and see if you can identify the nodes that you have just launched.

1. Finally, close down your publisher and subscriber nodes and the ROS Master by entering `Ctrl+C` in Terminals 1, 2 and 3.

## Launch Files

At the beginning of this session we launched our Gazebo Simulation and the `turtlebot3_teleop_keyboard` node using *launch files* and the `roslaunch` command. This provides a means to launch multiple ROS nodes *simultaneously*, and we will demonstrate this by building a launch file for the publisher and subscriber nodes that we created in the previous exercises.

#### Exercise 8: Creating a launch file {#ex8}

1. In **TERMINAL 1**, use `roscd` to navigate to the *root* of your `week1_pubsub` package directory.
1. Use the Linux `mkdir` command to **m**a**k**e a new **dir**ectory in the package root folder called `launch`:

    ***
    **TERMINAL 1:**
    ```bash
    mkdir launch
    ```
    ***

1. Use the `cd` command to enter the `launch` folder that you just created, then use the `touch` command (as before) to create a new empty file called `pubsub.launch`.
1. Open this launch file in VS Code and enter the following text:

    ```xml
    <launch>
      <node pkg={BLANK} type={BLANK} name="pub_node" output="screen">
      </node>
    </launch>
    ```

    !!! warning "Fill in the Blanks!"
        Referring to what we learned about [the format of launch files](#package_attributes) earlier, replace each `{BLANK}` above with the correct text to launch the publisher node that you created in [Exercise 6](#ex6).
    
1. Use `roslaunch` to launch this file and test it out as it is (remember: `roslaunch {package name} {launch file}`). If everything looks OK then carry on to the next step.
1. The code that we've given you above will launch the `publisher.py` node, but not the `subscriber.py` node.  Add another `<node>` tag to your `pubsub.launch` file to launch the subscriber node as well.
1. The publisher and subscriber nodes and the *ROS Master* can now all be launched with the `roslaunch` command and the `pubsub.launch` file that you have now created.  
1. Launch this in **TERMINAL 1** and then use `rosnode list` in **TERMINAL 2** to check that it all works correctly.

##### Summary {#roslaunch}

* `roslaunch` can be used to launch multiple nodes on a robot from *one single command*.
* It will *also* automatically launch the ROS Master (equivalent to running the `roscore` command manually) if it isn't already running (*did you notice that we didn't have to do this manually in Exercise 8, but we did when we launched our nodes individually, using `rosrun`, in Exercises 6 & 7?*)
* In the `rospy.init(...)` functions of our `publisher.py` and `subscriber.py` Python scripts, we defined a node name and set `anonymous=True`. As a result, when we launched our nodes manually using `rosrun`, the names we defined were honoured, but were appended with a unique combination of numbers.
* When we launched our nodes using `roslaunch` however, the node names were set according to what we had defined in the `name` field of the `<node>` tag within the launch file, and anything specified within the `rospy.init(...)` functions of our Python scripts were overwritten as a result.

## Wrapping Up

In this session we've learnt about some key concepts in ROS, such as Packages; Launch files; Nodes and the *Publisher-Subscriber Communication Method* using Topics and Messages.

We've learnt how to use some key ROS commands:  

* `roslaunch`: to launch multiple ROS Nodes via launch files.
* `roscd`: to navigate to installed ROS packages using a package name alone.
* `rosnode`: to display information about active ROS Nodes.
* `rosrun`: to run executables within a ROS package.
* `rostopic`: to display information about *active* ROS topics.
* `rosmsg`: to display information about *all* ROS messages that are available to use in a ROS application.
* `roscore`: to launch the *ROS Master*: The baseline nodes and programs that are required for ROS to function.

In addition to this we've also learnt how to use `catkin_create_pkg`, which is a helper script for creating ROS package templates.

We have also learnt how to work in the Linux Terminal and navigate a Linux filesystem using key commands such as:

* `pwd`: prints the path of the current working directory to show you which directory you're currently located in.
* `ls`: lists the files in the current directory.
* `cd`: change directory to move around the file system.
* `mkdir`: make a new directory (`mkdir {new_folder}`).
* `cat`: show the contents of a file.
* `chmod`: modify file permissions (i.e. to add execute permissions to a file for all users: `chmod +x {file}`).
* `touch`: create a file without any content.

Finally, we have learnt how to create basic ROS nodes in Python to both *publish* and *subscribe* to ROS topics using standard ROS messages.

### Saving your work {#backup}

Remember, the work you have done in the WSL-ROS environment during this session **will not be preserved** for future sessions or across different University machines automatically! To save the work you have done here today you should now run the following script in any idle WSL-ROS Terminal Instance:

```bash
wsl_ros backup
```

This will export your home directory to your University U: Drive, allowing you to restore it at the start of the next session.  
