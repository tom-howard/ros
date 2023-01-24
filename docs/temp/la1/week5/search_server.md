---
title: Week 5 Search Server Template
---

# Week 5 Search Server Template

```py title="search_server.py Template"
--8<-- "code/search_server.py"
```

!!! warning "Important"
    The template above uses the `tb3.py` module from the `tuos_ros_examples` package, which contains various helper functions to make the robot move and to read data (a.k.a. *subscribe*) from some key topics that will be useful for the task at hand. To use this, you'll need to copy the module across to your own `week5_actions/src` folder so that your `search_server.py` node can import it. In **TERMINAL 2**, copy the `.py` file as follows:

    ```bash
    cp ~/catkin_ws/src/COM2009/tuos_ros_examples/src/tb3.py ~/catkin_ws/src/week5_actions/src/
    ```

<p align="center">
  <a href="../../week5#ex4_ret">&#8592; Back to Week 5 - Exercise 4</a>
</p>