---  
title: Exporting your ROS package for submission  
---  

# Exporting your ROS Package for Submission

When it comes to submission time, it's important that you follow the steps below carefully to create an archive of your ROS package correctly. We recommend that you do this from WSL-ROS on a University Managed Desktop Computer (rather than one of the robot laptops), so that you can save it to your University `U:` Drive.

1. First, navigate to the `catkin_ws/src` directory in a WSL-ROS terminal instance:

    ```bash
    cd ~/catkin_ws/src/
    ```

1. Then, use the `tar` command to create an archive of your package:

    ```bash
    tar -cvf /mnt/u/wsl-ros/com2009_team{}.tar com2009_team{}
    ```
    
    ... replacing `{}` with your own team number, of course!

    This will create the `.tar` archive in your own personal University `U:` Drive, which you can access using the Windows File Explorer...

1. In *Windows*, open up Windows Explorer, click "This PC" in the left-hand toolbar and locate your own personal `U:` Drive in the "Network locations" area.

1. In here there should be a `wsl-ros` folder, which should contain the `com2009_team{}.tar` file that you just created.

1. Submit this `.tar` file to Blackboard via the appropriate submission portal.
