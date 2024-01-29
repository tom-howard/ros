---
title: Accessing ROS for this Course
---

# Accessing ROS for this Course

In order to engage with [the COM2009 Course](../com2009/README.md) you'll need access to a ROS environment. [As discussed here](../about/robots.md#ros-version), the course is designed around the latest (and final) version of ROS 1: **Noetic**. 

ROS is a bit difficult to install, and is only really available on a small selection of Linux platforms ([as listed here](https://wiki.ros.org/noetic/Installation)), the most common choice being Ubuntu 20.04. It is also possible to install it on Windows 10 or 11 via the Windows Subsystem for Linux (WSL). In order to install or access ROS for this course, we recommend one of the following options. Click on the relevant link below to access the associated instructions:

## WSL-ROS (highly recommended)
    
We've created our own custom ROS (Noetic) and Ubuntu (20.04) environment for WSL specifically for this course, which we call **"WSL-ROS"**. The environment contains all the tools and ROS packages that you will need for COM2009. This is our recommended option, and there are two ways to access this: 

1. [**Install it** on your own Windows 10 or 11 machine](./wsl-ros/install.md) via the IT Services Software Download Service.
1. [**Access it** on a range of managed desktop computers across the University campus](./wsl-ros/on-campus.md).

## Manually Install ROS and all Required Dependencies

If you have access to an Ubuntu 20.04 installation (either running natively on a personal computer or via WSL) then you can install all the necessary software manually, and we have instructions for this too:

1. **Windows 10 or 11**: [Installing ROS in an Ubuntu 20.04 WSL distro](./manual/windows.md).
1. **Native Linux Install**: [Installing ROS and all necessary dependencies on Ubuntu 20.04](./manual/linux.md).

!!! note "Mac Users"
    Unfortunately we aren't currently aware of a reliable way to install ROS Noetic on a Mac, and we'd therefore recommend that you [access WSL-ROS on a University Campus machine](./wsl-ros/on-campus.md).
    
    If you do come across a workable solution though then please let us know! 