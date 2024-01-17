---  
title: Launching ROS  
---  

# Launching ROS

The first step is to launch ROS on the Waffle.

!!! info "Important"
    Launching ROS on the Waffle enables the *ROS Master*[^rosmaster]. The ROS Master **always** runs on the Waffle. It's therefore important to complete the steps on this page **in full** before you do anything else, otherwise the ROS Master will not be running, the robot's core functionality won't be active, and you won't be able to do anything with it! 

[^rosmaster]: What *is* the ROS Master!? We'll talk about that on the next page ([here](./exercises.md#ros_master))

## Step 1: Identify your Waffle

Robots are named as follows:

    dia-waffleX

... where `X` indicates the *'Robot Number'* (a number between 1 and 50). Make sure you know which robot you are working with, or check the label printed on top of it!

## Step 2: Pairing your Waffle to a Laptop

[As discussed earlier](./intro.md#laptops), you'll be provided with one of our Robotics Laptops to work with in the lab, and the robot needs to be paired with this in order for the two to work together.  

1. Open up a terminal instance on the laptop, either by using the ++ctrl+alt+t++ keyboard shortcut, or by clicking the Terminal App icon in the favourites bar on the left-hand side of the desktop:
    
    <figure markdown>
      ![](../images/laptops/bash_terminal_icon.svg){width=60px}
    </figure>

1. We'll use our purpose-built `waffle` CLI to handle the pairing process. Run this in the terminal by entering the following command to *pair* the laptop and robot:

    ***
    ```bash
    waffle X pair
    ```
    Replacing `X` with the number of the robot that you are working with.
    
    ***

1. You may see a message like this early on in the pairing process:

    <figure markdown>
      ![](../images/laptops/ssh_auth.svg){width=600px}
    </figure>

    If so, just type `yes` and then hit ++enter++ to confirm that you want to continue.

1. Enter the password for the robot when requested (we'll tell you what this is in the lab!)

    !!! note
        You won't see anything change on the screen when you are entering the password. This is normal, just keep typing!!
    
1. Once the pairing process is finished you should see a message saying `pairing complete`, displayed in blue in the terminal. 

1. Then, in the same terminal, enter the following command:

    ***
    ```bash
    waffle X term
    ```
    (again, replacing `X` with the number of *your* robot).
    
    ***

    A green banner should appear across the bottom of the terminal window:
    
    <figure markdown>
      ![](../images/laptops/tmux.svg){width=700px}
    </figure>

    This is a terminal instance running **on the robot**, and any commands that you enter here will be executed *on the robot* (not the laptop!)

## Step 3: Launching ROS

Launch ROS (and the ROS Master) on the robot by entering the following command:

```bash
roslaunch tuos_tb3_tools ros.launch
```

??? tip
    To save you typing this command out in full all the time, we've created a handy *bash alias* for it! You can therefore use `tb3_bringup` instead.

After a short while, you should see a message like this:

``` { .txt .no-copy }
[INFO] [#####] --------------------------
[INFO] [#####] dia-waffleX is up and running!
[INFO] [#####] -------------------------- 
```

<center>

**ROS is now up and running on the robot, and you're ready to go!**

</center>

You shouldn't need to interact with this terminal instance any more now, but after about 20 seconds the screen will clear, and you'll be presented with some real-time info related to the status of the robot. Keep this terminal instance open in the background and keep an eye on the `Voltage` indicator in particular:

``` { .txt .no-copy } 
Voltage: 12.40V [100%]
```

!!! info "Low Battery :material-battery-low:"

    **The robot's battery won't last a full 2-hour lab session!!**

    When the capacity indicator reaches around 15% then let a member of the teaching team know and we'll replace the battery for you. (It's easier to do it at this point, rather than waiting until it runs completely flat.)

## At the End of Each Lab Session

When you've finished working with a robot it's really important to **shut it down properly** before turning off the power switch. Please refer to the [safe shutdown procedures](./shutdown.md) for more info.