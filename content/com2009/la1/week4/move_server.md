+++  
title = "Week 4 Move Service-Server Python Example"  
hidden = "true"
+++

#### The Code {#code}

Copy **all** the code below into your `move_server.py` file.  Then, review [the explainer below](#explainer) to understand how this all works.

{{< include file="move_server.py" code="true" lang="python" >}}

{{< nicenote warning "Fill in the Blank!" >}}
Which message package does [the `Twist` message](../../week2/#twist-py) belong to?
{{< /nicenote >}}

#### The Code Explained {#explainer}

Firstly, **don't forget the shebang**:

```python
#!/usr/bin/env python3
```

1. As you should know by now, in order to develop any ROS node in Python we first need to import the `rospy` library so that we can interact with ROS. We're also going to be issuing velocity commands to the robot, so we need to import the `Twist` message from the correct *message package* as well.

1. Then, we also import the *Service Message* that we want to use for the service that we will set up. This service will use the `SetBool` service message from a custom `tuos_ros_msgs` package that we've created for you:

    ```python
    from tuos_ros_msgs.srv import SetBool, SetBoolResponse
    ```

    Here, we import two different things from the `tuos_ros_msgs` package:

    1. A definition of the *full service message*: `SetBool`, which we need to use when we create the service later.
    1. The **Response** portion of the service message: `SetBoolResponse`, which we will use to issue a response to the service caller.

1. Next, we set up a publisher to the `/cmd_vel` topic, so that we can publish velocity commands to the robot (using `Twist` messages). Hopefully this part is starting to become familiar to you by now!

    ```python
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    ```

1. Then we define a `callback_function`:

    ```python
    def callback_function(service_request):
    ```

    (We'll come back to this in more detail in a bit.)

1. First, let's jump to the part of the code after the `callback_function()`, since this next bit will actually execute *first*:

    ```python
    rospy.init_node(f"{service_name}_server")
    my_service = rospy.Service(service_name, SetBool, callback_function)
    rospy.loginfo(f"the '{service_name}' Server is ready to be called...")
    rospy.spin()
    ```

    Here, we initialise a new ROS Node and call it `"move_service_server"`, via the `service_name = "move_service"` variable (inside [an f-string](https://realpython.com/python-f-strings/)). Then, we create a `rospy.Service` instance where we define:

    1. The *name* of the service that this node will launch (`service_name = "move_service"` at the beginning of the code).
    1. The full *service message format* that the service will use, in this case: `SetBool`, which we imported earlier.
    1. A *callback function*, in this case called `callback_function`, which will define what we want this service Server to do once the service is called.

    After this, we send some information to the terminal using `rospy.loginfo` to indicate that the node has been launched successfully and is ready to be called.  Finally, we use the `rospy.spin()` tool to make sure that our node keeps running indefinitely.

1. Now, let's return to the definition of the `callback_function()`. This contains the code that will be executed when the service is called. The function can take one input argument only, in this case we are calling it `service_request`. This is where the `rospy.Service` instance that we set up earlier will put the data that it obtains from a `/move_service` call, whenever a **Request** is made. 
    
    1. Inside this, we create an instance of the **Response** portion of the `SetBool` service message, which we will populate with data later on (based on the outcome of the actions that the service server performs).

        ```python
        def callback_function(service_request):

            service_response = SetBoolResponse()
        ```

    1. We then analyse the service **Request** data (this is the data that is passed to the Server node, whenever a call to the service is made by a caller, or *client*). We know how to access the data within the service request from using the `rossrv info` command, which provides us with the following information:

        ```txt
        rossrv info tuos_ros_msgs/SetBool:

        bool request_signal
        ---
        bool response_signal
        string response_message
        ```

        The **Request** message will contain a boolean value called `request_signal`, so we can call this value from the input to our callback function, which we called `service_request`. We will check if this value is `True` or `False` using an `if` statement, and then define some actions for each situation accordingly:

        ```python
        if service_request.request_signal == True:
        ```

    1. If the `service_request.request_signal` value is `True`, then we perform the following actions:

        1. Print a status message to tell the Service *caller* that a `True` value has been received:
            
            ```python
            print(f"The '{service_name}' Server received a 'true' request and the robot will now move for 5 seconds...")
            ```
    
        1. Get the current ROS time using the `rospy.get_rostime()` function:
            
            ```python
            StartTime = rospy.get_rostime()
            ```
    
        1. Set a linear velocity for the robot, publish this to the `/cmd_vel` topic using the publisher that we set up earlier and then issue another status message to indicate that this has been done:

            ```python
            vel.linear.x = 0.1
            pub.publish(vel)

            rospy.loginfo('Published the velocity command to /cmd_vel')
            ```

        1. Setup a while loop to act as a timer, and wait until 5 seconds has elapsed before doing anything else:

            ```python
            while (rospy.get_rostime().secs - StartTime.secs) < 5:
                continue
            ```
    
        1. Once the time *has* elapsed, issue another status message to indicate this, then publish another velocity command to make the robot stop:

            ```python
            rospy.loginfo('5 seconds have elapsed, stopping the robot...')

            vel.linear.x = 0.0
            pub.publish(vel)
            ```

        1. Finally, we can format a service **Response** using the `SetBoolResponse` instance that we set up earlier (called `service_response`). Again, we know how to access the fields within the service response from the `rossrv info` command as shown earlier, and thus we know that our **Response** should be constructed as follows:

            ```python
                service_response.response_signal = True
                service_response.response_message = "Request complete."
            ```

    1. If, however, the value of the `service_request.request_signal` was actually found to be `False` by our `if` statement earlier then we format our service response differently:

        ```python
        else:
            service_response.response_signal = False
            service_response.response_message = "Nothing happened, set request_signal to 'true' next time."
        ```