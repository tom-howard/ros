---  
title: Tips & Tricks  
---

## Bash Aliases

*Bash aliases* are abbreviations for long terminal commands. As you'll know, some common ROS commands that we use when working with the robots are pretty long, so we've created a few aliases to make it quicker to launch certain things. Most of these are for commands that you'll run on the laptop, but there are a couple that apply to the robot too. See the table below for the full list of bash aliases that are available to you when working with the robots and the laptops:

<center>

| Alias | Full Command | Context |
| :--- | :--- | :--- |
| `tb3_teleop` | `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` | Laptop |
| `tb3_bringup` | `roslaunch turtlebot3_bringup turtlebot3_remote.launch` | Laptop |
| `tb3_slam` | `roslaunch turtlebot3_slam turtlebot3_slam.launch` | Laptop |
| `tb3_rviz` | `roslaunch tuos_tb3_tools rviz.launch` | Laptop |
| `tb3_world` | `roslaunch turtlebot3_gazebo turtlebot3_world.launch` | Laptop |
| `tb3_sim` | `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch` | Laptop |
| `tb3_bringup` | `roslaunch tuos_tb3_tools ros.launch` | Robot |
| `src` | `source ~/.bashrc` | Both |

</center>

## "Simulation Mode" on the Laptops

By default, the laptops are set up to work with the real robots, but it is possible to switch them into *"Simulation Mode"* in order to work with ROS and the Waffle in simulation instead. All the simulations that you've been working with in WSL-ROS are available to launch on the laptops, once you're in simulation mode. 

To switch into Simulation Mode, enter the following command:

```bash
robot_mode sim
```

... which should present you with the following message:

```txt
Switching into 'Simulation Mode' (run 'robot_mode robot' to work with a real robot).
```

!!! note
    When you're ready to switch back to a real robot, the `waffle` CLI tool will switch you back into *"Real Robot Mode"* automatically! Just follow [the steps here](../launching-ros).