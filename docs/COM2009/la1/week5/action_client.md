---  
title: Week 5 Camera Sweep Action Client
---

# Week 5 Camera Sweep Action Client

Copy **all** the code below into your `move_client.py` file.  Then, review the code annotations to understand how it all works.

(Oh, and [DFTS](../../week1/subscriber/#dfts)!)

```py title="action_client.py"
--8<-- "code/action_client.py"
```

1. As you know by now, in order to develop ROS nodes using Python we need to use the `rospy` library. 

2. If we want to work with ROS Actions, we also need to import `actionlib`.

3. We know that the `/camera_sweep_action_server` uses `CameraSweepAction` messages from the `tuos_ros_msgs` package, so we import the full message definition: `CameraSweepAction` as well as the `Goal` message (which we use to actually make a call to the server). 

    As we now know, ROS Actions provide **feedback**, so we're also importing the `Feedback` part of the `CameraSweepAction` message into our client node as well, so that it can be kept up to date with what the Action Server is doing, in real-time.

4. To process this feedback data, we need to define a *feedback callback function*, to handle the data whenever a new feedback message comes in.

    Here we're using [Python's "Type Annotations"](https://docs.python.org/3/library/typing.html) feature:
    
    ```py
    feedback_data: CameraSweepFeedback
    ```
    ... which informs the interpreter that the `feedback_data` that is received by the `feedback_callback` function will be of the `CameraSweepFeedback` type.

    All this really does is allow autocomplete functionality to work with in our editor (VS Code), whenever we want to pull an attribute from the `feedback_data` object.

5. Standard practice when we construct ROS nodes: *we must initialise them with a name,* where the `node_name` variable was assigned earlier on in the code: 
    
    ```py
    node_name = "camera_sweep_action_client"
    ```

6. Then, we connect to the action server using the `actionlib.SimpleActionClient()` method, provide this with the name of the server that we wish to connect to and the type of messages that are used on the server (in this case `CameraSweepAction` messages).

    (The `action_server_name` variable was also assigned earlier on in the code too: `#!python action_server_name = "/camera_sweep_action_server"`)

7. This makes the node wait until the action server is live on the ROS Network (if it isn't already). Execution of the code can't continue past this point until the Action Server is visible on the ROS Network.

8. Once the server is available, we construct a **goal** message and send this to the action server, whilst also pointing it to the callback function that we defined earlier, which will be used to process the feedback messages.

    !!! tip
        Both goal parameters have been set to `0` (above), so you'll need to change these in order for the client to successfully make a call to the server!

9. Then, we simply wait for the action to complete.

10. Once it *has* completed, the server provides us with a **result**, so we simply print that (as well as the current **state** of the action) to the terminal. 

!!! warning "Fill in the Blank!"
    Which attribute of the `feedback_data` object tells us how many images have been captured over the course of the *Camera Sweep* Action? There are a number of ways we can work this out:
        
    1. You could use the same approach as we used [earlier](../#camera_sweep_msg_params). 
    1. You could run `rosmsg info tuos_ros_msgs/CameraSweepFeedback` in a terminal.
    1. You could use the autocomplete/variable suggestions provided in VS Code!

<p align="center">
  <a href="../../week5#ex2_ret">&#8592; Back to Week 5 - Exercise 2</a>
</p>