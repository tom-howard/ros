+++
menuTitle = "The Robots"
title = "Introducing the Robots"
weight = 1
+++

{{% textalign left %}}
[<i class="fas fa-solid fa-arrow-left"></i> Previous Page: "About These Courses"](/intro/)
{{% /textalign %}}

## The TurtleBot3 Waffle

### Turtlebot what?!

To teach ROS here we use the [TurtleBot3 Waffle](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot, made by Robotis. This is the 3rd Generation Robot in the [TurtleBot family](http://wiki.ros.org/Robots/TurtleBot) (which has been the reference hardware platform for ROS since 2010). The TurtleBot Robot family exists to provide accessible and relatively low-cost hardware and open-source software on a robot platform, to encourage people to learn robotics and ROS and make it as easy as possible to do so.

### The (Free) TurtleBot3 eBook

The TurtleBot3 Waffle developers (Robotis) have written a book on programming robots with ROS. This is available as a *free eBook*, which you can download [here](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51), and we recommend that you do so! This is a great resource which provides a detailed introduction to what ROS is and how it works, as well as a comprehensive "Beginners Guide" to ROS programming. The other great thing about this is that it is tailored to the TurtleBot3 Robot specifically, providing examples of how to use a range of TurtleBot3 packages along with a detailed description of how they work.

We recommend that you have a look at this book to learn more about the concepts that you are exploring in these courses.

### Our Waffles

Here in the Diamond we have a number of *customised* TurtleBot3 Waffles specifically for teaching this course:

![](/figures/waffle/cabinet.jpg?width=12cm) 

Whether we're working in simulation or with the real thing, the ROS applications that we develop as part of the course materials here are directly transferable between the two (mostly!) 

The robots that we have are in fact slightly different to the standard *TurtleBot3 WafflePi* model that you can buy directly from Robotis. We've made a few adjustments though, as shown below:

![](/figures/waffle/features.png?width=20cm)

The robots have the following core hardware elements:
* Independent left and right wheel motors (DYNAMIXEL XM430’s) to drive the robot using a *differential drive* configuration.
* An OpenCR Micro-Controller Board to power and control the wheel motors, distribute power to other hardware elements and provide an interface for additional sensors.
* An [UP Squared Single-Board Computer (SBC)](https://up-board.org/upsquared/specifications/) with an Intel Processor and 32GB of on-board eMMC storage. This board acts as the "brain" of the robot.

In addition to this, the robots are equipped with the following sensors:
* A Light Detection and Ranging (or *LiDAR*) sensor, which spins continuously when the robot is in operation. This uses light in the form of laser pulses to allow the robot to measure the distance to surrounding objects, providing it with a 360&deg; view of its environment.
* An [Intel RealSense D435 Camera](https://www.intelrealsense.com/depth-camera-d435/) with left and right imaging sensors, allowing depth sensing as well as standard image capture.
* A 9-Axis Inertial Measurement Unit (or *IMU*) on-board the OpenCR Micro Controller board, which uses an accelerometer, gyroscope and magnetometer to measure the robot's specific force, acceleration and orientation. 
* Encoders in each of the DYNAMIXEL wheel motors, allowing measurement of speed and rotation count for each of the wheels.

### ROS Version

Our robots run on the most up-to-date version of ROS1: [ROS Noetic Ninjemys](http://wiki.ros.org/noetic) (or *"ROS Noetic"* for short). Our courses here are therefore based around this version of ROS. ROS1 is best installed on [the Ubuntu Operating System](https://ubuntu.com/) for stability, reliability and ease, and ROS *Noetic* runs on [Ubuntu 20.04 (Focal Fossa)](https://wiki.ubuntu.com/FocalFossa/ReleaseNotes).

## Other Tech in the Diamond

### Laptops

In the Diamond, we have dedicated Robot Laptops running the same OS & ROS version as above (Ubuntu 20.04 and ROS Noetic). We use these for all of our real robot activities.

### Simulation Environment

To deliver the simulation-based parts of this course, we've created a custom simulation environment using the [Windows Subsystem for Linux (WSL)](https://docs.microsoft.com/en-us/windows/wsl/). This has been developed primarily to run on University of Sheffield Managed Desktop Computers, which run Windows 10, but it's also possible to run this on other machines too. We call this simulation environment *"WSL-ROS"*.

{{% textalign right %}}
[Next Page: "Acknowledgements" <i class="fas fa-solid fa-arrow-right"></i>](/intro/acknowledgements)
{{% /textalign %}}