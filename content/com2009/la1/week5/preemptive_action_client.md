+++  
title = "Week 5 Preemptive Action Client"  
hidden = "true"
+++

#### The Code {#code}

Copy **all** the code below into your `preemptive_action_client.py` file and then, review [the explainer](#explainer) to understand how this all works.

{{< include file="preemptive_action_client.py" code="true" lang="python" >}}

{{< nicenote warning "Fill in the Blank!" >}}
We have contained all our code inside a nice Python Class now, but how do we actually instantiate it and invoke the Action Call? (We've been doing this from [the very beginning](../week1/publisher), and the process is very much the same here!)
{{< /nicenote >}}

#### The Code Explained {#explainer}

Our imports are all exactly the same as last time here.  The main difference now though is that everything else is now contained within a Python Class:

```python
class preemptiveActionClient():
```
Just below this, we instantiate a **goal** message object, which will be populated later, when we want to call the action:

```python
    goal = CameraSweepGoal()
```

The *feedback callback function* is exactly the same, except we are now making the `captured_images` variable available across the whole class using the `self` prefix (previously this was achieved using the `global` statement).

```python
def feedback_callback(self, feedback_data: CameraSweepFeedback):
    self.captured_images = feedback_data.current_image
    print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
          f"Image(s) captured so far: {self.captured_images}...")
```

We now have an `__init__()` method inside the class, which will be executed as soon as the class is instantiated. Here, we do all our initialisations:
* Initialise some variables (`captured_images`, `action_complete`) and make them available throughout the class by assigning them to `self`.
* Initialise the node (with a name).
* Create a connection to the action server.
* Specify a function to be executed when the node is stopped (`shutdown_ops()`).

```python
def __init__(self):
    self.captured_images = 0
    self.action_complete = False

    node_name = "preemptive_camera_sweep_action_client"
    action_server_name = "/camera_sweep_action_server"
    
    rospy.init_node(node_name)

    self.rate = rospy.Rate(1)

    self.client = actionlib.SimpleActionClient(action_server_name, 
                CameraSweepAction)
    self.client.wait_for_server()

    rospy.on_shutdown(self.shutdown_ops)
```

The actual shutdown operations are then defined within the `shutdown_ops()` function.  This is how we make sure that the current goal is cancelled (using `cancel_goal()`), so that it doesn't just keep on running when this node is stopped prematurely (before the action has completed).  This function will also execute when the action server completes successfully, so we use an `action_complete` flag to check whether this is the case (i.e. to avoid trying to cancel the goal if it's already finished!):

```python
def shutdown_ops(self):
    if not self.action_complete:
        rospy.logwarn("Received a shutdown request. Cancelling Goal...")
        self.client.cancel_goal()
        rospy.logwarn("Goal Cancelled")
    
    # get the result:
    rospy.sleep(1) # wait for the result to come in
    print("RESULT:")
    print(f"  * Action State = {self.client.get_state()}")
    print(f"  * {self.captured_images} image(s) saved to {self.client.get_result()}")
```

The bit outside the `if` statement will execute regardless of whether the action completed successfully or was preempted. Here, we're getting the **result** from the action server and printing it to the terminal, along with the action state (using the `get_state()` method) and the final number of captured images (obtained from the last **feedback** message that was issued before the action stopped).

The way the goal is defined and issued to the server is exactly the same as before, except this time it's done within a class method, so that it can be called from `main_loop()`:

```python
def send_goal(self, images, angle):
    self.goal.sweep_angle = angle
    self.goal.image_count = images
    
    # send the goal to the action server:
    self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
```

Inside the `main_loop()` method, we then call the goal and then do exactly the same as we did before: monitor the **state** of the action with a `while` loop and do some concurrent operations (seven-times tables again!).

```python
def main_loop(self):
    self.send_goal(images = 10, angle = 90)
    i = 1
    print("While we're waiting, let's do our seven-times tables...")
    while self.client.get_state() < 2:
        print(f"STATE: Current state code is {self.client.get_state()}")
        print(f"TIMES TABLES: {i} times 7 is {i*7}")
        i += 1
        self.rate.sleep()
    self.action_complete = True
```

The key difference here is that we set the `action_complete` flag to `True` if the action manages to complete successfully.

Finally, at the very bottom we need to actually launch the action server by instantiating the class and running the right class method from within it (taking care to do some error checking at the same time...

```python
if __name__ == '__main__':
    ...
```