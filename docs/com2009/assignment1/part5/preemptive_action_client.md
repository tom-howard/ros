---  
title: Part 5 Preemptive Action Client
---

# Part 5 Preemptive Action Client

Copy **all** the code below into your `preemptive_action_client.py` file and then review the annotations!

```py title="preemptive_action_client.py"
--8<-- "snippets/preemptive_action_client.py"
```

1. Everything is now contained within a Python Class.

2. Instantiate a **goal** message object, which we'll use later to call the Camera Sweep action.

3. The *feedback callback function* is exactly the same as the one in the `action_client.py` from Exercise 2. Because we're working inside a Python Class now, we can make the `captured_images` variable available beyond the scope of this function by using the `self` prefix (previously this was achieved using the `global` statement).

4. Classes require an `__init__()` method, which will be executed as soon as the class is instantiated. Here, we do all our initialisations:

    * Initialise some variables (`captured_images`, `action_complete`) and make them available throughout the class by prefixing with `self`.
    * Initialise the node (with a name).
    * Set a rate (1 Hz).
    * Create a connection to the action server and wait for it to become available.
    * Specify a function to be executed when the node is stopped (`shutdown_ops()`).

    ... none of this should be new to you now!

5. The actual shutdown operations are defined here. This is how we make sure that the current goal is cancelled (using `cancel_goal()`), so that the action doesn't keep on running if this node is stopped prematurely (before the action has completed).  This function will also execute when the action server completes successfully, so we use an `action_complete` flag to check whether this is the case (i.e. to avoid trying to cancel the goal if it's already finished!):

6. This bit will execute regardless of whether the action completed successfully or was preempted. Here, we're getting three things and printing them to the terminal:
    
    1. The **result** from the action server.
    1. The action state (using the `get_state()` method).
    1. The final number of captured images (obtained from the last **feedback** message that was issued before the action stopped).

7. The way the goal is defined and issued to the server is exactly the same as before, except this time it's done within this class method, so that it can be called from `main()`.

8. Call the goal.

9. All this is the same as before (`action_client.py`), i.e. monitor the **state** of the action with a `while` loop and do some concurrent operations (seven-times tables again!).

10. The only difference is that we set this flag to `True` if the action manages to complete successfully.

11. **Fill in the Blank!**

!!! warning "Fill in the Blank!"
    We have contained all our code inside a nice Python Class now, but how do we actually instantiate it and invoke the Action Call? (We've been doing this from [the very beginning](../part1/publisher.md), and the process is very much the same here!)

<p align="center">
  <a href="../../part5#ex3_ret">&#8592; Back to Part 5 - Exercise 3</a>
</p>