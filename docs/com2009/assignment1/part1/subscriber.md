---  
title: Part 1 Subscriber Node  
---

## The Code

Copy **all** the code below into your `subscriber.py` file and (again) *make sure you read the annotations to understand how it all works!*

```python title="subscriber.py"
--8<-- "snippets/subscriber.py"
```

1. As with our publisher node, we need to import the `rospy` client library and the `String` message type from the `std_msgs.msg` library in order to write a Python ROS Node and use the relevant ROS messages:

2. This time, we create a Python Class called `Subscriber()` instead.

3. When building a subscriber, we need a *callback function*. Within this function, we define what we want to do with the messages that we obtain from the topic we're listening (*subscribing*) to:

    In this case, we simply want to print the `String` message to the terminal.

4. Here we define the initialisation operations for the class:

    1. Here, we firstly initialise a rospy node with a custom name (in the same way as we initialised the publisher node earlier). 
    1. Then, we create a `rospy.Subscriber` object, set this to listen to the topic that we want to receive messages from, specify the message type used by this topic, and then define the *callback function* to use to process the data whenever a message comes in.
    1. Then, we send a message to the terminal to indicate that our node is running.

5. The `rospy.spin()` method simply makes sure that our `main()` keeps running until the node is shut down externally (i.e. by a user entering `Ctrl+C`).

6. Finally, the code is executed by again performing a `__name__` check, creating an instance of the `Subscriber()` class and calling the `main()` method from that class.

<a name="blank-1"></a>

!!! warning "Fill in the Blank!"
    Replace the `{BLANK}` in the code above with the name of the topic that our [`publisher.py` node](./publisher.md) was set up to publish to!

## Don't Forget the Shebang! {#dfts}

First, **don't forget the shebang**, it's very important!

```python
#!/usr/bin/env python3
```

## A Simpler Approach

The above code uses a Python *Class structure*.  This approach will be very useful when we start to do more complex things later in the course, but for this exercise you could also achieve the same using the following simplified approach:

```python
--8<-- "snippets/subscriber_alt.py"
```

<p align="center">
  <a href="../../part1#ex6_ret">&#8592; Back to Part 1 - Exercise 6</a>
</p>