---  
title: Exporting your ROS Package for Submission  
---  

# Exporting your ROS Package for Submission

When it comes to submission time, it's important that you follow the steps below carefully to create an archive of your ROS package correctly. We recommend that you do this from WSL-ROS, or your own local ROS installation, rather than one of the Robotics Laptops.

1. First, run the following command, which will create a folder in your home directory called `myrosdata` (if it doesn't already exist):

    ```bash
    mkdir -p ~/myrosdata/
    ```

2. Then, navigate to your `catkin_ws/src` directory:

    ```bash
    roscd && cd ../src/
    ```

3. Use the `tar` command to create an archive of your team's package:

    ```bash
    tar -cvf ~/myrosdata/com2009_team{}.tar com2009_team{}
    ```
    
    ... replacing `{}` with your own team number, of course!

    This will create a `.tar` archive of your package in the `~/myrosdata` folder. 

4. If you're using WSL-ROS (or any other WSL-based ROS installation) then you can then access this using the Windows File Explorer. In the terminal enter the following command:

    ```bash
    explorer.exe ~/myrosdata
    ```

5. An Explorer window should open, and in there you should see the `com2009_team{}.tar` file that you just created. Copy and paste this to somewhere convenient on your machine.

6. Submit this `.tar` file to the appropriate submission portal on Blackboard.
