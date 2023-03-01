---  
title: "Getting Started: Some Initial Exercises"  
---

# Getting Started: Some Initial Exercises

Having completed the steps on [the previous page](../launching-ros), your robot and laptop should now be paired, and ROS should be up and running. The next thing to do is bring the robot to life! Here are a few simple exercises for you to have a go at, to get a feel for things... 

#### :material-pen: Exercise 1: Observing the robot's environment {#ex1}

You can use some ROS tools that you will be familiar with from simulation in order to see the *real* world through the eyes of the robot!

1. Open up a new terminal instance on the laptop (which we'll call **TERMINAL 1**). From here, enter the following command:

    ***
    **TERMINAL 1:**
    ```bash
    roslaunch tuos_tb3_tools rviz.launch
    ```

    ??? tip "Pro Tip"
        We've got a handy bash alias for this command: use `tb3_rviz` instead (if you want to)!
    
    ***

    This will launch *RViz*, which (as you may recall from your simulation work) is a ROS tool that allows us to *visualise* the data being measured by a robot *in real-time*.
    
    <figure markdown>
      ![](../images/laptops/waffle_rviz.png?width=20cm)
    </figure>

    * The red dots scattered around the robot is the data from the LiDAR sensor.

    * The strange wobbly sheet of colours in front of the robot is the live image stream from the camera with depth applied to it at the same time. The camera is able to determine how far away each image pixel is from the camera lens, and then uses that to render the image in 3-dimensions. Nice eh!?

1. Place your hand or your face in front of the camera and hold steady for a few seconds (there may be a bit of a lag as all of this data is transmitted over the WiFi network). You should see yourself rendered in 3D in front of the robot!

1. Another tool that you'll be familiar with from Week 6 is `rqt_image_view`. Again, we can use this to view live image data being streamed to ROS image topics. Open another terminal instance on the laptop (**TERMINAL 2**) and launch this as follows:

    ***
    **TERMINAL 2:**
    ```bash
    rosrun rqt_image_view rqt_image_view
    ```
    ***

    Select `/camera/color/image_raw` in the dropdown topic list to see the images being obtained and published by the robot's camera.

Use these tools to keep an eye on your robot's environment whilst performing the next exercise...

#### :material-pen: Exercise 2: Driving the robot around using the laptop keyboard {#ex2}

We used the `turtlebot3_teleop_keyboard` node extensively in simulation to drive a Waffle around a range of simulated environments. This works in exactly the same way with a real robot in a real world!

1. Open another new terminal instance (**TERMINAL 3**) and enter exactly the same `roslaunch` command as you've used in simulation to launch the `turtlebot3_teleop` node:

    ***
    **TERMINAL 3:**
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
    
    ??? tip "Pro Tip"
        There's a bash alias for this one too: `tb3_teleop`!

    ***

1. Drive the robot around using the laptop keyboard (as you did in simulation) taking care to avoid any obstacles as you do!

1. Once you've spent a bit of time on this, close the teleop node down by entering `Ctrl+C` in **TERMINAL 3**.

1. Close down RViz and the `rqt_image_view` nodes running in **TERMINAL 2** and **TERMINAL 1** as well, we won't need these for the next exercise.

#### :material-pen: Exercise 3: Creating a ROS package on the Laptop {#ex3}

This works exactly the same as in simulation too.

A Catkin Workspace exists on the *laptop's* filesystem here:

    /home/student/catkin_ws/

a.k.a.:

    ~/catkin_ws/

!!! warning "Remember"
    Much like in simulation, **always** create ROS packages in the `catkin_ws/src/` directory!

1. In **TERMINAL 1** navigate to the Catkin Workspace `src` directory:

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/catkin_ws/src/
    ```
    ***

1. Create a new package here using the `catkin_create_pkg` tool. **Don't forget to include `rospy` as a dependency**! 

    ***
    **TERMINAL 1:**
    ```bash
    catkin_create_pkg {your_package} rospy
    ```
    ***

    ??? tip "Using Git?"
        Already created a ROS package in simulation? Why not push it to GitHub (or GitLab, etc.) and `git clone` it here instead?

1. Next, don't forget to run `catkin build`:

    ***
    **TERMINAL 1:**
    ```bash
    catkin build {your_package}
    ```
    ***

1. Then, re-source your environment:

    ***
    **TERMINAL 1:**
    ```bash
    source ~/.bashrc
    ```
    
    ??? tip "Pro Tip"
        You can also use the `src` alias, just like in simulation!
    
    ***

#### :material-pen: Exercise 4: Using SLAM to create a map of the environment {#ex4}

Remember how you used SLAM in Week 3 to create a map of a simulated environment? We'll do this now on a *real robot* in a *real environment*!

1. In **TERMINAL 2** enter the following command to launch all the necessary SLAM nodes on the laptop:

    ***
    **TERMINAL 2:**
    ```bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch
    ```
    
    ??? tip "Pro Tip"
        *Also* available as an alias: `tb3_slam`!

    ***

    This will launch RViz again, where you should now be able to see a model of the Waffle from a top-down view surrounded by green dots representing the real-time LiDAR data. The SLAM tools will already have begun processing this data to start building a map of the boundaries that are currently visible to your robot based on its location in the environment.

    !!! note
        To begin with your robot may just appear as a white shadow (similar to the left-hand image below). It may take some time for the robot to render correctly (like the right-hand image) as the SLAM processes and data communications catch up with one another. 
        
        <figure markdown>
          ![](../images/waffle/slam.png){width=600px}
        </figure>
        
        This can sometimes take up to a minute or so, so please be patient! If (after a minute) nothing has happened, then speak to a member of the teaching team.

1. Return to **TERMINAL 3** and launch the `turtlebot3_teleop_keyboard` node again. Start to drive the robot around *slowly* and *carefully* to build up a complete map of the area.

    !!! tip
        It's best to do this slowly and perform multiple circuits of the whole area to build up a more accurate map.

1. Once you're happy that your robot has built up a good map of its environment, you can save this map using the `map_server` package (again, in exactly the same way as you did in simulation):

    1. First, create a new directory within your `{your_package}` package on the laptop (to save maps in). You should still be in your package directory in **TERMINAL 1**, so head back to that one:

        1. There's no harm in running this, just to make sure that you're in the right place to begin with:

            ***
            **TERMINAL 1:**
            ```bash
            roscd {your_package}
            ```
            ***

        1. Create a directory in here called `maps`: 
            
            ***
            **TERMINAL 1:**
            ```bash
            mkdir maps/
            ```
            ***

        1. Navigate into this directory:

            ***
            **TERMINAL 1:**
            ```bash
            cd maps/
            ```
            ***

    1. Then, use `rosrun` to *run* the `map_saver` node from the `map_server` package to save a copy of your map:

        ***
        **TERMINAL 1:**
        ```bash
        rosrun map_server map_saver -f {map_name}
        ```
        
        Replacing `{map_name}` with an appropriate name for your map. This will create two files: 
        
        1. a `{map_name}.pgm` 
        1. a `{map_name}.yaml` file
        
        ...both of which contain data related to the map that you have just created.

        ***


    1. The `.pgm` file can be opened in `eog` on the laptop: 
    
        ***
        **TERMINAL 1:**
        ```bash
        eog {map_name}.pgm
        ```
        ***

1. Return to **TERMINAL 2** and close down SLAM by pressing `Ctrl+C`. The process should stop and RViz should close down.

1. Close down the `teleop` node in **TERMINAL 3**, if that's still going too.


