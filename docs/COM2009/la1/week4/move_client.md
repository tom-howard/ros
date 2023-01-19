---  
title: "Week 4 Move Service-Client Python Example"  
---

#### The Code {#code}

Copy **all** the code below into your `move_client.py` file and review the annotations to understand how it all works.

!!! note "DFTS!!"
    (Don't forget the shebang!)
    ```python
    #!/usr/bin/env python3
    ```

```py title="move_client.py"
--8<-- "code/move_client.py"
```

1. Again, the first step when building a Python node is to import the `rospy` library so that Python and ROS can interact. 

2. This service client will use the `SetBool` service message from the `tuos_ros_msgs` package, so we import the full definition of the `SetBool` Service Message, as well as *the portion of the message that we will need to use to actually issue a service call*.

3. We import the Python `sys` module to do some error handling for us,  we don't need to worry about too much.

4. Define the name of the service that we want to call, and assign this to a variable called `service_name` (for convenience, since we'll refer to this a couple of times). 

5. Initialise the client node (give it a name).

6. Wait until the service that we want to call is actually running, execution of this node will not progress beyond this point until the service is detected on the ROS network (launched by the Server).

7. Once it *is* running, we create a connection to it and specify the service message type that it uses (as defined above).

8. Create an instance of the `{BLANK}` part of the service message, and populate this with the data that the server is expecting.

9. **Remember:** Using `rossrv info` on this service message in a terminal tells us the attribute names for both the **Request** and **Response**:

    ```bash
    rossrv info tuos_ros_msgs/SetBool
    ```
    ...gives us the following:
    ```txt
    bool request_signal
    ---
    bool response_signal
    string response_message
    ```

10. Use the `rospy.ServiceProxy` instance that we created earlier (called `service`) to actually send the `service_request` message to the service and obtain a response back from the Server (once it's complete).

11. To finish off, we print the response to the terminal to give the user some feedback. Job done!

!!! warning "Fill in the Blank!"
    Consider the `import` statement for [the service *Server* that we created earlier](../move_server)... Which part of the `SetBool` Service message was imported here? Now consider that you need to build a client to call this service... which part of the `SetBool` Service message is needed in order to *call* a service? 

    **Note:** the same `{BLANK}` appears in two places in the code above - *the answer is the same in both places!*

<p align="center">
  <a href="../../week4#ex2_ret">&#8592; Back to Week 4 - Exercise 2</a>
</p>