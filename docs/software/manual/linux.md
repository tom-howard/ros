---
title: Installing ROS on Linux (Ubuntu 20.04)
---

# Installing ROS on Linux (Ubuntu 20.04)

**Applicable to**: Those who have access to a computer running Ubuntu 20.04 (either natively or via WSL).

If you have Ubuntu 20.04 running on a machine then follow the steps below to install ROS and all the additional packages required for this ROS Course:

1. Install ROS Noetic (the instructions that follow are largely taken from [the ROS.org website](http://wiki.ros.org/noetic/Installation/Ubuntu)):

    1. Set up your system to accept software from packages.ros.org:

        ```bash
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        ```

    1. Set up your keys (using `curl`): 

        ```bash
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        ```

    1. Make sure your package index is up-to-date:

        ```bash
        sudo apt update
        ```

    1. Install the "Desktop-Full" version of ROS:

        ```bash
        sudo apt install ros-noetic-desktop-full
        ```

1. Set up your environment.

    1. A ROS script must be sourced in every bash terminal you use ROS in, so it's best to add a line to the end of your `~/.bashrc` so that this occurs automatically whenever you open a terminal:

        ```bash
        source /opt/ros/noetic/setup.bash
        ```

    1. Re-source your environment for the changes to take effect:

        ```bash
        source ~/.bashrc
        ```

1. Install dependencies for building packages

    > "Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command."

    ```bash
    sudo apt install python3-rosdep \
    python3-rosinstall \ 
    python3-rosinstall-generator \ 
    python3-wstool build-essential
    ```

1. Initialize `rosdep`:

    > "Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS."

    ```bash
    sudo rosdep init
    ```
    ```bash
    rosdep update
    ```

1. Install *'Catkin Tools'*. This is optional. By default, `catkin_make` can be used to invoke CMake for building ROS packages, but we use `catkin build` instead (because it's a bit nicer!) [See how to install Catkin Tools here](https://catkin-tools.readthedocs.io/en/latest/installing.html), if you want to. 

1. Create and build a Catkin Workspace:

    1. Make the directories:

        ```bash
        mkdir -p ~/catkin_ws/src
        ```
    
    1. Navigate into the root folder:

        ```bash
        cd ~/catkin_ws/
        ```
    
    1. Then **if you installed Catkin Tools**:

        ```bash
        catkin build
        ```
    
        ... and if not, then its:

        ```bash
        catkin_make
        ```
    
    1. Then you need to add this to the end of your `~/.bashrc` as well:

        ```bash
        source ~/catkin_ws/devel/setup.bash
        ```
    
    1. And finally, re-source:

        ```bash
        source ~/.bashrc
        ```

1. Install the TurtleBot3 packages, based on [the Robotis instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-ros-on-remote-pc):

    1. First, install dependencies:

        ```bash
        sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
        ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
        ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
        ros-noetic-rosserial-python ros-noetic-rosserial-client \
        ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
        ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
        ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
        ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
        ```

    1. Then, install the TurtleBot3 packages themselves:

        ```bash
        sudo apt install ros-noetic-dynamixel-sdk
        ```
        ```bash
        sudo apt install ros-noetic-turtlebot3-msgs
        ```
        ```bash
        sudo apt install ros-noetic-turtlebot3
        ```
        ```bash
        sudo apt install ros-noetic-turtlebot3-simulations
        ```

    1. And then add some environment variables to your `~/.bashrc` too, in order for ROS and the TurtleBot3 packages to launch correctly:

        ```bash
        export TURTLEBOT3_MODEL=waffle
        export ROS_MASTER_URI=http://localhost:11311
        export ROS_HOSTNAME=localhost
        ```

1. Next, install some other useful Python tools:

    ```bash
    sudo apt install python3-pip python3-scipy python3-pandas
    ```

2. Finally, install [the Course Repo](https://github.com/tom-howard/tuos_ros.git):

    1. Navigate to your Catkin Workspace:

        ```bash
        cd ~/catkin_ws/src/
        ```

    2. Download the repo from GitHub:

        ```bash
        git clone https://github.com/tom-howard/tuos_ros.git
        ```
        
    3. Then, **if you installed Catkin Tools**:

        ```bash
        catkin build
        ```

        ... **if not**, do this:

        ```bash
        cd ~/catkin_ws/ && catkin_make
        ```
    
    4. And finally, re-source again:

        ```bash
        source ~/.bashrc
        ```
   
3. For convenience, we use some *Bash Aliases* to make it easier to call some common ROS commands. You might want to create a `~/.bash_aliases` file with the following content (or add to an existing one):

    ```bash
    alias tb3_teleop="rosrun turtlebot3_teleop turtlebot3_teleop_key"
    alias tb3_world="roslaunch turtlebot3_gazebo turtlebot3_world.launch"
    alias tb3_empty_world="roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch"
    alias tb3_slam="roslaunch turtlebot3_slam turtlebot3_slam.launch"
    alias tb3_rviz="roslaunch tuos_simulations rviz.launch"

    # This one's quite useful too:
    alias src="echo 'Sourcing bashrc...' && source ~/.bashrc"
    ```