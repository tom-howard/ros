---  
title: "Part 4 Move Service-Client"  
---

# Part 4 Move Service-Client

Copy **all** the code below into your `move_client.py` file and review the annotations to understand how it all works.

!!! note "DFTS!!"
    (Don't forget the shebang!)
    ```python
    #!/usr/bin/env python3
    ```

```py title="move_client.py"
--8<-- "snippets/move_client.py"
```

1. Again, the first step when building a Python node is to import the `rospy` library so that Python and ROS can interact. 

2. This service client will use the `SetBool` service message from the `tuos_msgs` package, so we import the full definition of the `SetBool` Service Message, as well as *the portion of the message that we will need to use to actually issue a service call*.

3. Define the name of the service that we want to call, and assign this to a variable called `service_name` (for convenience, since we'll refer to this a couple of times). 

4. Initialise the client node (give it a name).

5. Wait until the service that we want to call is actually running, execution of this node will not progress beyond this point until the service is detected on the ROS network (launched by the Server).

6. Once it *is* running, we create a connection to it and specify the service message type that it uses (as defined above).

7. Create an instance of the `{BLANK}` part of the service message, and populate this with the data that the server is expecting.

8. **Remember:** Using `rossrv info` on this service message in a terminal tells us the attribute names for both the **Request** and **Response**:

    ```bash
    rossrv info tuos_msgs/SetBool
    ```
    ...gives us the following:
    ```txt
    bool request_signal
    ---
    bool response_signal
    string response_message
    ```

9. Use the `rospy.ServiceProxy` instance that we created earlier (called `service`) to actually send the `request_to_server` message to the service and obtain a response back from the Server (once it's complete).

10. To finish off, we print the response to the terminal to give the user some feedback. Job done!

<a name="blank-2"></a>

!!! warning "Fill in the Blank!"
    Consider the `import` statement for [the service *Server* that we created earlier](./move_server.md)... Which part of the `SetBool` Service message was imported here? Now consider that you need to build a client to call this service... which part of the `SetBool` Service message is needed in order to *call* a service? 

    **Note:** the same `{BLANK}` appears in two places in the code above - *the answer is the same in both places!*

<p align="center">
  <a href="../../part4#ex2_ret">&#8592; Back to Part 4 - Exercise 2</a>
</p>