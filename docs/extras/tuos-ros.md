---  
title: "The TUoS ROS Course Repo"
--- 

A ROS Metapackage called `tuos_ros` has been put together to support these courses. [This package is available on GitHub here](https://github.com/tom-howard/tuos_ros.git). This repo contains the following ROS packages:

| Package Name | Description |
| :---: | :--- |
| `tuos_examples` | Some example scripts to support certain exercises in COM2009 Assignment #1 |
| `tuos_msgs` | Some custom ROS messages to support certain exercises in COM2009 Assignment #1 ([to learn how to create your own custom ROS messages see here](./ros-msgs.md)) |
| `tuos_simulations` | Some Gazebo simulations to support certain exercises in COM2009 Assignment #1 |
| `tuos_tb3_tools` | Scripts that run on the real Waffles, and some RViz configs to support real robot work too | 
| `com2009_simulations` | Simulation resources to support your development work in COM2009 Assignment #2 |

## Installing

The `tuos_ros` course repo is already installed and ready to go in WSL-ROS, for those who use it, and on the Robotics Laptops. To install the packages in your own local ROS installation, follow the steps here.

1. Make sure you've already set up a Catkin Workspace in your home directory.
1. Navigate to your Catkin Workspace `src` directory:

    ```bash
    cd ~/catkin_ws/src/
    ```

1. Clone the repo from GitHub:

    ```bash
    git clone https://github.com/tom-howard/tuos_ros.git
    ```

1. Run `catkin build` to compile the packages:

    ```bash
    catkin build
    ```

1. Then re-source your `.bashrc`:

    ```bash
    source ~/.bashrc
    ```

### Verify

Once you've installed it, then you can verify that the build process has worked using the following command:

```bash
roscd tuos_ros
```

Which should take you to a directory within the repo that's also called `tuos_ros`, i.e.:

```txt
~/catkin_ws/src/tuos_ros/tuos_ros
```

## Updating

The course repo may be updated every now and again, so its worth checking regularly that you have the most up-to-date version. You can do this by pulling down the latest updates from GitHub using `git pull`:

```bash
roscd && cd ../src/tuos_ros/ && git pull
```

If you see the following message:

```txt
-bash: cd: tuos_ros/: No such file or directory
```

... then [go back and make sure you've installed the repo first](#installing)!

Then, run `catkin build` and re-source your environment:

```bash
roscd && cd .. && catkin build && source ~/.bashrc
```