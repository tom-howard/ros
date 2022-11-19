+++  
title = "Week 5 Camera Sweep Action Client"  
hidden = "true"
+++

#### The Code {#code}

Copy **all** the code below into your `move_client.py` file.  Then, review [the explainer below](#explainer) to understand how this all works.

{{< include file="action_client.py" code="true" lang="python" >}}

{{< nicenote warning "Fill in the Blank!" >}}
Which attribute of the `feedback_data` object tells us how many images have been captured over the course of the *Camera Sweep* Action? There are a number of ways we can work this out:
1. You could use the same approach as we used [earlier](../#camera_sweep_msg_params). 
1. You could run `rosmsg info tuos_ros_msgs/CameraSweepFeedback` in a terminal.
1. You could use the autocomplete/variable suggestions provided in VS Code!
{{< /nicenote >}}

#### The Code Explained {#explainer}

[DFTS, of course](../week1/subscriber/#dfts)!

As you know by now, in order to develop ROS nodes using Python we need to use the `rospy` library.  If we want to work with ROS Actions, we also need to import `actionlib`. We know that the `/camera_sweep_action_server` uses `CameraSweepAction` messages from the `tuos_ros_msgs` package, so we import the full message definition: `CameraSweepAction` as well as the `Goal` message (which we use to actually make a call to the server). 

```python
import rospy
import actionlib

from tuos_ros_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback
```

As we now know, ROS Actions provide **feedback**, so we're also importing the `Feedback` part of the `CameraSweepAction` message into our client node as well, so that it can be kept up to date with what the Action Server is doing, in real-time.

To process this feedback data, we need to define a *feedback callback function*, to process the data whenever a new feedback message comes in:

```python
captured_images = 0
def feedback_callback(feedback_data: CameraSweepFeedback):
    global captured_images
    captured_images = feedback_data.{BLANK}
    print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
          f"Image(s) captured so far: {captured_images}...")
```

Here we're using [Python's "Type Annotations"](https://docs.python.org/3/library/typing.html) feature:

```python
def feedback_callback(feedback_data: CameraSweepFeedback):
    ...
```

Which informs the interpreter that the `feedback_data` that is received by the `feedback_callback` function will be of the `CameraSweepFeedback` type. All this really does is allow autocomplete functionality to work with in our editor (VS Code), whenever we want to pull an attribute from the `feedback_data` object.

As is standard practice when we construct ROS nodes, we must initialise them with a node name:

```python
rospy.init_node(node_name)
```

Where the `node_name` variable was assigned earlier on in the code as:

```python
node_name = "camera_sweep_action_client"
```

Then, we connect to the action server using the `actionlib.SimpleActionClient()` method, provide this with the name of the server that we wish to connect to and the type of messages that are used on the server (in this case `CameraSweepAction` messages):

```python
client = actionlib.SimpleActionClient(action_server_name, 
            CameraSweepAction)
```

Where the `action_server_name` variable was also assigned earlier on in the code too:

```python
action_server_name = "/camera_sweep_action_server"
```

After that, the node simply waits until the action server is activated (if it isn't already):

```python
client.wait_for_server()
```

Once the server is available, we construct a **goal** message and send this to the action server, whilst also pointing it to the callback function that we defined earlier, which will be used to process the feedback messages:

```python
goal = CameraSweepGoal()
goal.sweep_angle = 0
goal.image_count = 0

client.send_goal(goal, feedback_cb=feedback_callback)
```

{{% nicenote tip %}}
Both goal parameters are set to `0` here, so you'll need to change these in order for the client to successfully make a call to the server!
{{% /nicenote %}} 

Then, we simply wait for the action to complete, by using the `wait_for_result()` method.  Once it *has* completed, the server provides us with a **result**, so we simply print that (as well as the current **state** of the action) to the terminal:

```python
client.wait_for_result()

print(f"RESULT: Action State = {client.get_state()}")
print(f"RESULT: {captured_images} images saved to {client.get_result()}")
```