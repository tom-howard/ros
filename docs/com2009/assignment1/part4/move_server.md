---  
title: "Part 4 Move Service-Server"  
---

# Part 4 Move Service-Server

Copy **all** the code below into your `move_server.py` file and review the annotations to understand how it all works.

!!! note "Remember"
    **Don't forget the shebang**!

    ```python
    #!/usr/bin/env python3
    ```

```py title="move_server.py"
--8<-- "snippets/move_server.py"
```

1. As you should know by now, in order to develop any ROS node in Python we first need to import the `rospy` library so that we can interact with ROS. We're also going to be issuing velocity commands to the robot, so we need to import the `Twist` message from the correct *message package* as well.

2. We also need to import the *Service Message* that we want to use for the service that we will set up. This service will use the `SetBool` service message from a custom `tuos_msgs` package that we've created for you.

    Here, we import two different things from the `tuos_msgs` package:

    1. A definition of the *full service message*: `SetBool`, which we need to use when we create the service later.
    1. The **Response** portion of the service message: `SetBoolResponse`, which we will use to issue a response to the service caller.

3. Here, we set up a publisher to the `/cmd_vel` topic, so that we can publish velocity commands to the robot (using `Twist` messages). Hopefully this part is starting to become familiar to you by now!

4. Here we define a callback function for the server called `srv_callback`. Any code within this function will be executed whenever the service is called.

    The function can take one input argument only, in this case we are calling it `request_from_client`. This is where the `rospy.Service` instance that we set up earlier will put the data that it obtains from a `/move_service` call, whenever a **Request** is made.

5. We create an instance of the **Response** portion of the `SetBool` service message, which we will populate with data later on (based on the outcome of the actions that the service server performs).

6. We then analyse the service **Request** data (this is the data that is passed to the Server node, whenever a call to the service is made by a caller, or *client*). We know how to access the data within the service request from using the `rossrv info` command, which provides us with the following information:

    ```txt
    rossrv info tuos_msgs/SetBool:

    bool request_signal
    ---
    bool response_signal
    string response_message
    ```

    The **Request** message will therefore contain a boolean value called `request_signal`, so we can call this value from the input to our callback function (which we called `request_from_client`). Using an `if` statement, we check if this value is `True` or `False`, and then define some actions for each situation accordingly...

7. Print a status message to tell the Service *caller* that a `True` value has been received.

8. Get the current ROS time.

9. Set a linear velocity for the robot, publish this to the `/cmd_vel` topic using the publisher that we set up in the `__init__()` method earlier (`self.pub`).

10. Here, we use a while loop to act as a 5-second timer (by keeping an eye on the current ROS time using `get_rostime()`). Once 5 seconds have elapsed, this while loop will end.

11. Once the time *has* elapsed, we publish another velocity command to make the robot stop.

12. Finally, we can format a service **Response** using the `SetBoolResponse` instance that we set up earlier (`response_from_server`). Again, we know the names of the attributes in the service response from the `rossrv info` command:

    ```txt
    rossrv info tuos_msgs/SetBool:

    bool request_signal
    ---
    bool response_signal
    string response_message
    ```

13. If the value of the `request_from_client.request_signal` was actually found to be `False` by our `if` statement earlier, then we do nothing other than send a service response, to indicate that nothing has happened!

14. Initialise our ROS Node with a name (`"move_service_server"`).

15. Create a `rospy.Service` instance where we define:

    1. The *name* of the service that this node will launch (`service_name = "move_service"` at the beginning of the `__init__()` method).
    1. The full *service message format* that the service will use, in this case: `SetBool`, which we imported earlier.
    1. A *callback function*, in this case called `srv_callback`, which will define what we want this service Server to do once the service is called.

16. Send some information to the terminal to indicate that the node has been launched successfully, and that the Service is ready to be called.

17. The `rospy.spin()` function keeps our node running indefinitely (so that the callback function can continue to execute, whenever the service is called). 

!!! warning "Fill in the Blank!"
    Which message package does [the `Twist` message](../part2/twist-tips.md) belong to?

<p align="center">
  <a href="../../part4#ex1_ret">&#8592; Back to Part 4 - Exercise 1</a>
</p>