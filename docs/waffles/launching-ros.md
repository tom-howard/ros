---  
title: Launching ROS  
---  

# Launching ROS

The first step is to launch ROS on the Waffle.

!!! info "Important"
    Launching ROS on the Waffle enables the *ROS Master*. The ROS Master **always** runs on the Waffle. It's therefore important to complete the steps on this page **in full** before you do anything else, otherwise the ROS Master will not be running, the robot's core functionality won't be active, and you therefore won't be able to do anything with it! 

## Step 1: Identify your Waffle

Robots are named as follows:

    dia-waffleX

... where `X` indicates the *'Robot Number'* (a number between 1 and 50). Make sure you know which robot you are working with, or check the label printed on top of it!

## Step 2: Pairing your Waffle to a Laptop

[As discussed earlier](../intro/#laptops), you'll be provided with one of our Robotics Laptops to work with in the lab, and the robot needs to be paired with this in order for the two to work together.  

1. Open up a terminal instance on the laptop, either by using the `Ctrl+Alt+T` keyboard shortcut, or by clicking the Terminal App icon in the favourites bar on the left-hand side of the desktop:
    
    <figure markdown>
      ![](../images/laptops/bash_terminal_icon.svg){width=60px}
    </figure>

1. We'll use our purpose-built *Waffle CLI* to handle the pairing process. Run this in the terminal by entering the following command to *pair* the laptop and robot:

    ***
    ```bash
    waffle X pair
    ```
    Replacing `X` with the number of the robot that you are working with.
    
    ***

1. You *may* see a message like this early on in the pairing process:

    <figure markdown>
      ![](../images/laptops/ssh_auth.svg){width=600px}
    </figure>

    If so, just type `yes` and then hit `Enter` to confirm that you want to continue.

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

    This is a terminal instance running *on the robot*, and any commands that you enter here will be *executed* on the robot (not the laptop!)

## Step 3: Launching ROS

Launch ROS (and the ROS Master) on the robot by entering the following command:

***
```bash
roslaunch tuos_tb3_tools ros.launch
```
***

??? tip "Pro Tip"
    To save you typing this command out in full all the time, we've created a handy *bash alias* for it! You can therefore use `tb3_bringup` instead.

After a short while, you should see a message like this:

```txt
[INFO] [#####] Calibration End  
```

<center>

**ROS is now up and running, and you're ready to go!**

</center>

You shouldn't need to interact with this terminal instance any more now. You can either keep it open in the background (to keep an eye on any status messages coming through), or you can close down the terminal instance entirely. If you want to close it down then hit the :material-close-circle: button in the top right of the terminal window. You'll then see the following message:

<figure markdown>
  ![](../images/laptops/term_confirm_close.png){width=600px}
</figure>

... just click "Close Terminal."

!!! warning
    When you've finished working with a robot it's really important to **shut it down properly** before turning off the power switch. Please refer to the [safe shutdown procedure](../shutdown) detailed here.