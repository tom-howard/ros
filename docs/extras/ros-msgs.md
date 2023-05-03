---  
title: "Creating Your Own Custom ROS Messages"  
---

# Creating Your Own Custom ROS Messages

*Want to create your own ROS Topic/Service/Action Messages?*

Defining your own messages is quite straight forward, but there are a few key rules to follow, and these rules differ slightly depending on whether you are trying to create a standard **topic message** type, a **Service** message or an **Action**. You'll also need to make sure that the `CMakeLists.txt` and `package.xml` files within your ROS package contain the correct instructions so that the necessary source code is generated correctly when you run `catkin build` (i.e. so that you can import the messages into your Python (or C++ etc.) nodes).

Below are some links to some useful external resources that explain how to create each different type of ROS message for your own specific applications:

* [Creating ROS Topic and Services Messages (`msg` and `srv`)](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* [Creating ROS Action Messages](https://roboticsbackend.com/ros-create-custom-action/)
