+++  
title = "Week 4 Move Service-Client Python Example"  
hidden = "true"
+++

#### The Code {#code}

Copy **all** the code below into your `move_client.py` file.  Then, review [the explainer below](#explainer) to understand how this all works.

{{< include file="move_client.py" code="true" lang="python" >}}

{{< nicenote warning "Fill in the Blank!" >}}
Consider the `import` statement for [the service *Server* that we created earlier](../move_server)... Which part of the `SetBool` Service message was imported here? Now consider that you need to build a client to call this service... which part of the `SetBool` Service message is needed in order to *call* a service? 

**Note:** the same `{BLANK}` appears in two places in the code above - *the answer is the same in both places!*
{{< /nicenote >}}

#### The Code Explained {#explainer}

**DFTS!!** (*Don't forget the shebang!*):
```python
#!/usr/bin/env python3
```

1. Again, the first step when building a Python node is to import the `rospy` library so that Python and ROS can interact. This service client will use the `SetBool` service message from the `tuos_ros_msgs` package, so we then need to import the full definition of the `SetBool` Service Message, as well as *the portion of the message that we will need to use to actually issue a service call*. Finally, we import the Python `sys` module for some error handling, which we don't need to worry about too much.

1. After the imports we define the name of the service that we want to call, and assign this to a variable called `service_name` (we do this for convenience, since we'll refer to this a couple of times). 

    ```python
    service_name = "move_service"
    ```

1. We also use this to name our node when initialising it:

    ```python
    rospy.init_node(f"{service_name}_client")
    ```

1. We use the `rospy.wait_for_service()` method to wait until the service that we want to call is actually running:

    ```python
    rospy.wait_for_service(service_name)
    ```

1. Once it *is* running, we create a connection to it and specify the service message type that is used by it:

    ```python
    service = rospy.ServiceProxy(service_name, SetBool)
    ```

1. We then create an instance of the `{BLANK}` part of the service message, and then populate this with the data that we need to provide to the Server in order to make the service call.

    ```python
    service_request = {BLANK}()
    service_request.request_signal = True
    ```

    **Remember:** Using `rossrv info` on this service message in a terminal:

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

1. Finally, we use the `rospy.ServiceProxy` instance that we created earlier (which was assigned to the variable `service`) to actually send the `service_request` message to the service and obtain a response back from the Server, once it's complete. To finish off, we print the response to the terminal to give the user some feedback:

    ```python
    service_response = service(service_request)
    print(service_response)
    ```

Job done.