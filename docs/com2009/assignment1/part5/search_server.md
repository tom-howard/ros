---
title: Part 5 Search Server Template
---

# Part 5 Search Server Template

```py title="search_server.py Template"
--8<-- "snippets/search_server.py"
```

!!! warning "Important"
    The template above uses the `tb3.py` module from the `tuos_examples` package, which contains various helper functions to make the robot move and to read data (a.k.a. *subscribe*) from some key topics that will be useful for the task at hand. To use this, you'll need to copy the module across to your own `part5_actions/src` folder so that your `search_server.py` node can import it. In **TERMINAL 2**, copy the `.py` file as follows:

    ```bash
    cp ~/catkin_ws/src/tuos_ros/tuos_examples/src/tb3.py ~/catkin_ws/src/part5_actions/src/
    ```

<p align="center">
  <a href="../../part5#ex4_ret">&#8592; Back to Part 5 - Exercise 4</a>
</p>