---  
title: "The WSl-ROS Simulation Environment"  
---  

# The WSL-ROS Simulation Environment

To support these courses we've created a custom ROS (Noetic) and Ubuntu (20.04) environment which runs on Windows 10 using the [Windows Subsystem for Linux (WSL)](https://docs.microsoft.com/en-us/windows/wsl/). We call this **"WSL-ROS"**, and you can find out more about it - and how to use it - here.

WSL-ROS is installed on all the University of Sheffield Managed Desktop Computers located in **Computer Rooms 1, 2, 3 & 4 in the Diamond**. It's also possible to access it on *some* remote-access machines (see ["Accessing the Environment Remotely"](rdp) for more details).

Each time you launch WSL-ROS on an applicable University Machine the WSL-ROS environment will be installed from a custom OS image that we have created, which contains the Ubuntu 20.04 operating system, ROS and all the additional ROS packages that you will need for this lab course. The environment that you install will only remain on the machine that you install it on for a limited time, and won't be preserved across different computers that you log into. It's therefore really important that you [follow some backup and restore procedures as detailed below](backup-restore).