+++  
title = "Week 1 Subscriber Node"  
hidden = "true"
+++

## The Code

Copy **all** the code below into your `subscriber.py` file. Numbers inside square brackets in the code refer to the numbered items in [the explainer below](#explainer)... *make sure you read this to understand how it all works!*

{{< include file="subscriber.py" code="true" lang="python" >}}

{{% nicenote warning "Fill in the Blank!" %}}
Replace the `{BLANK}` in the code above with the name of the topic that our [`publisher.py` node](Week-1-Publisher-Node) was set up to publish to! 
{{% /nicenote %}}

## The Code Explained {#explainer}

1. First, **don't forget the shebang!**

    ```python
    #!/usr/bin/env python3
    ```

1. As with our publisher node, we need to import the `rospy` client library and the `String` message type from the `std_msgs.msg` library in order to write a Python ROS Node and use the relevant ROS messages:

    ```python
    import rospy
    from std_msgs.msg import String
    ```

1. Next, we create a `Subscriber` class:

    ```python
    class Subscriber():
    ```

1. Within our class, we firstly create a *callback function*, for the `rospy.Subscriber` method to use later. Within this function, we define what we want to do with the messages that we obtain from the topic we are listening to:

    ```python
    def callback(self, topic_message):
        print(f"Subscriber obtained the following message: '{topic_message.data}'")
    ```

    In this case, we simply want to format the `String` message and print it out to the terminal.

1. Next, we define our initialisation operations for the class:

    ```python
    def __init__(self):
        node_name = "simple_subscriber"
        topic_name = {BLANK}

        rospy.init_node(node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, String, self.callback)
    ```

    Here, we firstly initialise a rospy node with a custom name (in the same way as we initialised the publisher node earlier). Then, we create a `rospy.Subscriber` object, set this to listen to the topic that we want to receive messages from, specify the message type used by this topic, and then define the function which will be called when a message is received. The received message will automatically be set as the first input to this function, so we need to make sure that the `self.callback()` function is configured to accept the message data as its first input (after `self`).

1. Then, we send a message to the terminal to indicate that our node is active:

    ```python
    rospy.loginfo(f"The '{node_name}' node is active...")
    ```

1. Then, we define the functionality of the main loop, which will run continuously until the node is shut down:

    ```python
    def main_loop(self):
        rospy.spin()
    ```

    `rospy.spin()` simply keeps the node running until it is shutdown externally (i.e. by a user entering `Ctrl+C`).

1. Finally, the code is executed by again performing the standard Python name check, creating an instance of the `Subscriber` class and calling the `main_loop()` class method:

    ```python
    if __name__ == '__main__':
        subscriber_instance = Subscriber()
        subscriber_instance.main_loop()
    ```

## An Alternative Implementation

The above code uses a Python *Class structure*.  This approach will be very useful when we start to do more complex things later in the course, but for this exercise you could also achieve the same using the following simplified approach:

{{< include file="subscriber_alt.py" code="true" lang="python" >}}