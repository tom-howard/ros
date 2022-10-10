+++  
title = "Week 1 Publisher Node"  
hidden = "true"
+++

## The Code

Copy **all** the code below into your `publisher.py` file.  Then, review [the explainer below](#explainer) to understand how it all works.

{{< include file="publisher.py" code="true" lang="python" >}}

## The Code Explained {#explainer}

First things first, the very first line in this file looks like a comment, but it is actually a very crucial part of the script:

```python
#!/usr/bin/env python3
```

This is called the *Shebang* and it tells the operating system which interpreter to use to execute the code. In our case here, it tells the operating system where the Python interpreter is installed and then uses this interpreter to actually run the code. **Make sure that this exists as the first line in all the Python nodes that you create in ROS!**

Then on to the actual python code. `rospy` is the *Python client library for ROS* and we need to import this in order to write a ROS Node in Python. We also need to import the `String` message type from the `std_msgs.msg` library for publishing our messages:

```python
import rospy
from std_msgs.msg import String
```

Next, we create a *Publisher class* to contain all the functionality of our node:

```python
class Publisher():
```

The `__init__()` method is called as soon as an instance of the Publisher class is created:

```python
def __init__(self):
    self.node_name = "simple_publisher"
    topic_name = "chatter"

    self.pub = rospy.Publisher(topic_name, String, queue_size=10)
    rospy.init_node(self.node_name, anonymous=True)
```

In the first part of this method we define a name for this node (`self.node_name`). We can call the node anything that we want, but it's good to give it a meaningful name as we will register the node on the ROS Network using this. We also define the name of a topic that we want to publish messages to (`"chatter"` in this case). If this is the name of a topic that already exists on the ROS Network, then we need to ensure that we use the correct message type, in order to be able to publish to it. In this case however, there shouldn't a topic called "chatter" shouldn't currently exists so it will be created for us, and we can choose whatever type of message we want to use (a `String` message from the `std_msgs` library in our case).

We then create an instance of a `rospy.Publisher()` object within our class: this is where the topic actually gets created on the ROS Network (if it doesn't already exist). We therefore need to tell it the name of the topic that we want to create and we also need to specify that we will be publishing `String` type messages. Then, we initialise our Python node with the name that we defined earlier (`"simple_publisher"`), setting the `anonymous` flag to `True` to ensure that the node is unique, which is achieved by adding random numbers to the end of it (we will observe this when we run the node shortly).

Then, we create a `Rate` object and set the frequency to 10 Hz, so that our publisher will publish messages at this frequency.

```python
self.rate = rospy.Rate(10) # hz
```

The next section of the initialisation block illustrates a nice way to shutdown a ROS node effectively:

```python
self.ctrl_c = False
rospy.on_shutdown(self.shutdownhook)
```

First, we create a `ctrl_c` variable within the parent class and initialise it to `False`. Then, we use the `rospy.on_shutdown()` method to register a `shutdownhook` function, which will be called when rospy begins to shutdown (i.e. the user has entered `Ctrl+C` in the terminal, or rospy has stopped for another reason). **This function must take no arguments.**

Finally, we issue a message to indicate that our node is active (this will appear in the terminal that we run the node in):

```python
rospy.loginfo(f"The '{self.node_name}' node is active...")
```

Next, we define the `shutdownhook()` function:

```python
def shutdownhook(self):
    print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
    self.ctrl_c = True
```

Here, we can define any important shutdown processes that may need to take place before the node actually stops (making a robot stop moving, for instance). In this case, we just print a message to the terminal. After this, we set the `ctrl_c` variable to `True`, which will stop the `main_loop()` method...

The `main_loop()` method contains a `while` loop, allowing it to run continuously until the node is shut down (via the `self.ctrl_c` flag):

```python
def main_loop(self):
    while not self.ctrl_c:
        publisher_message = f"rospy time is: {rospy.get_time()}"
        self.pub.publish(publisher_message)
        self.rate.sleep()
```

Inside the `while` loop we create a publisher message (a simple string in this case), publish it using the `pub` object we created in the initialisation stage, and then use the `rate` object (also created earlier) to then make the node "sleep" for as long as required to satisfy the frequency that we defined earlier.

Finally, the code is actually executed by first performing the standard python `__name__` check, to ensure that the script being run is the main executable (i.e. it has been executed directly, and hasn't been called by another script):

```python
if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
```

Then, we create an instance of the `Publisher` class that we created above (at which point the `__init__` method will be called). Following this, we call the `main_loop()` method to execute the core functionality of our class until further notice. We use a `try-except-pass` statement to look for a `rospy.ROSInterruptException` error, which can be output by rospy when the user presses `Ctrl+C` or the node is shutdown in some other way.