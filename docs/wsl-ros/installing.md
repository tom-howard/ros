---  
title: "Installing ROS on your own PC"  
---

We've set up the WSL-ROS environment specifically for these ROS Courses, to ensure that you have all the right packages and tools available to you. Of course, this requires you to work on a University Managed Computer and - naturally - you may want to be able to work through this course material (and explore further) on your own device instead.

## Quick Links

* [Installing ROS and Course Dependencies on Ubuntu 20.04](#install-ros)
* [Installing on Windows using WSL](#wsl)
    * [Running Graphical Applications in WSL on Windows 10](#running-graphical-applications-in-wsl-on-windows-10)

## Installing ROS and Course Dependencies on Ubuntu 20.04 {#install-ros}

If you already have Ubuntu 20.04 running on a machine then follow the steps below to install ROS and all the additional packages required for these ROS Courses:

1. Install ROS Noetic (the instructions that follow are largely taken from [the ROS.org website](http://wiki.ros.org/noetic/Installation/Ubuntu)):

    1. Set up your computer to accept software from packages.ros.org:

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

    1. A script must be sourced in every bash terminal you use ROS in, so it's best to add a line to the end of your `~/.bashrc`:

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
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
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

    1. Install Pip:

        ```bash
        sudo apt install python3-pip
        ```

    1. And use Pip to install [Pandas](https://pandas.pydata.org/docs/index.html):

        ```bash
        pip3 install pandas
        ```

1. Finally, install [the Course Repo](https://github.com/tom-howard/COM2009):

    1. Navigate to your Catkin Workspace:

        ```bash
        cd ~/catkin_ws/src/
        ```

    1. Download the repo from GitHub:

        ```bash
        git clone https://github.com/tom-howard/COM2009.git
        ```
        
    1. Then, **if you installed Catkin Tools**:

        ```bash
        catkin build
        ```

        ... **if not**, do this:

        ```bash
        cd ~/catkin_ws/ && catkin_make
        ```
    
    1. And finally, re-source again:

        ```bash
        source ~/.bashrc
        ```
   
1. For convenience, we use some *Bash Aliases* to make it easier to call some common ROS commands. You might want to create a `~/.bash_aliases` file with the following content (or add to an existing one):

    ```bash
    alias tb3_teleop="roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
    alias tb3_world="roslaunch turtlebot3_gazebo turtlebot3_world.launch"
    alias tb3_empty_world="roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch"
    alias tb3_slam="roslaunch turtlebot3_slam turtlebot3_slam.launch"
    alias tb3_rviz="roslaunch tuos_ros_simulations rviz.launch"

    # This one's quite useful too:
    alias src="echo 'Sourcing bashrc...' && source ~/.bashrc"
    ```

## Installing on Windows using WSL {#wsl}

Refer to [the Windows Subsystem for Linux Documentation](https://docs.microsoft.com/en-us/windows/wsl/install) for instructions on how to install and use WSL on Windows 10 or 11.

Ubuntu 20.04 should be installed by default when you install WSL but if not, or if you want to create an additional WSL distribution then [see here](https://docs.microsoft.com/en-us/windows/wsl/install#change-the-default-linux-distribution-installed).

We'd also recommend installing [the Windows Terminal App](https://docs.microsoft.com/en-us/windows/terminal/install).

Launch your Ubuntu 20.04 distro and then follow [the steps for Installing ROS (and dependencies)](#install-ros) above.

Graphical Applications are only supported natively in WSL when running on Windows 11 so if you're running Windows 10, then you will need to follow the additional steps below to get GUI apps (such as Gazebo and RViz) working...

### Running Graphical Applications in WSL on Windows 10

First, you'll need to install the [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/). You'll need to make sure you have this running before trying to launch any GUI applications from WSL (Gazebo simulations etc.) To make this easier, we've created [a configuration file](https://drive.google.com/file/d/19_ScBc8rVHwXTR2CRogwHArYqCc0ZAbD/view?usp=sharing). Download this file, save it on your desktop and double click it to launch an X Server on your machine with the appropriate configurations. Once launched, an icon should be visible in your notification tray in the bottom right-hand corner of the Windows Desktop:

<figure markdown>
  ![](../images/wsl/xlaunch_icon.png)
</figure>

In Ubuntu, you'll need to then add the following lines to your `~/.bashrc`:

```bash
export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf 2>/dev/null):0
export LIBGL_ALWAYS_INDIRECT=
export GAZEBO_IP=127.0.0.1
```
