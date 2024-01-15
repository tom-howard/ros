---
title: Introduction
---

# Introduction

## Handling the Robots

!!! warning "Health and Safety"
    You must have completed a health and safety quiz before working with the robots for the first time. This quiz is available on Blackboard.

<figure markdown>
  ![](../images/waffle/features_hl.png){width=600px}
</figure>

As you can see from the figure above, the robots have lots of exposed sensors and electronics and so you must take great care when handling them to avoid the robots becoming damaged in any way.  When handling a robot, always hold it by either the black **Waffle Layers**, or the vertical **Support Pillars** (as highlighted in the figure above).

!!! important 
    Do not pick the robot up or carry it by the camera or LiDAR sensor! These are delicate devices that could be easily damaged!

A lot of the electronics are housed on the middle waffle layer. Try not to touch any of the circuit boards, and take care not to pull on any of the cabling or try to remove or rehouse any of the connections. If you have any concerns with any of the electronics or cabling, if something has come loose, or if your robot doesn't seem to be working properly then ask a member of the teaching team to have a look for you.

The robots will be provided to you with a battery already installed and ready to go. **Don't try to disconnect or remove the battery yourselves**! The robot will beep when the battery is low, and if this happens ask a member of the team to get you a replacement (we have plenty).

## The Robotics Laptops {#laptops}

You'll be provided with one of our pre-configured *Robotics Laptops* in the lab when working with the real Waffles. These Laptops (and the Robots) run **Ubuntu 20.04** with **ROS Noetic**. 

On the laptops there is a "student" user account that you'll use when working in the lab. The laptop should log you into this user account automatically on startup, but we'll provide you with the account password as well, during the lab sessions, should you need it.

### WiFi {#dialab}

The Robots and Laptops connect to a dedicated wireless network running in the Diamond called *'DIA-LAB'*. There are a few things that you need to know about this:

* Laptops must be connected to the DIA-LAB network in order to establish a ROS network between them and the robots.
* Laptops **do not have internet access** when connected to DIA-LAB.
* You'll need to connect the laptop to *eduroam* (or use another computer) to access any external resources (such as the instructions on this site).

Credentials for DIA-LAB and eduroam have already been set on the laptops, allowing you to connect to either network straight away, but speak to a member of the teaching team if you are having any issues.

### IDE: VS Code

*Visual Studio Code* is installed on the laptops for you to use when working on your ROS applications for the assignment tasks. Launch VS Code from any terminal by simply typing `code`. You can also launch it by clicking the icon in the favourites bar on the left-hand side of the screen:

<figure markdown>
  ![](../images/vscode/icon.png)
</figure>
