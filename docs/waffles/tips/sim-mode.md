---  
title: Running the Laptops in Simulation Mode  
---

By default, the laptops are set up to work with the real robots, but it is possible to switch them into *"Simulation Mode"* in order to work with ROS and the Waffle in simulation instead. All the simulations that you work with in Assignment #1 are available to launch on the laptops, once you're in simulation mode. 

To switch into Simulation Mode, enter the following command:

```bash
robot_mode sim
```

... which should present you with the following message:

``` { .txt .no-copy }
Switching into 'Simulation Mode' ...
```

!!! note
    When you're ready to switch back to a real robot, the `waffle` CLI tool will switch you back into *"Real Robot Mode"* automatically! Just follow [the steps here](../launching-ros.md).