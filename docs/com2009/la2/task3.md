---  
title: "Task 3: Maze Navigation"  
---  

Develop the ROS node(s) that allow a simulated TurtleBot3 Waffle robot to navigate a maze in 150 seconds or less without crashing into anything.

**Assessment Format**: Simulation  
**Marks**: 15/100

## Summary

For this task your robot will need to navigate a maze of corridors without touching the walls.

As with Task 2, the LiDAR sensor will be essential here, allowing the robot to detect the maze walls and maintain a safe distance from them. Your robot will need to progress through the maze all the way to the end in order to be awarded maximum marks for this. The robot's *Odometry System* might prove useful too; allowing you to keep track of where your robot is and where it has already been, so that it continues to progress through the maze and doesn't waste time going back on itself. The maze will also contain **dead-ends**, so you'll need to make sure your robot is able to navigate its way out of these and (ideally) not go down them repeatedly!

One common method for solving mazes is to use a [wall following algorithm](https://en.wikipedia.org/wiki/Maze-solving_algorithm#Wall_follower). The maze that your simulated robot will be placed in **won't** contain any islands, i.e. it will be *"simply connected"*, so this is one method you might choose to adopt here.

## Details

As with Task 2, the simulated arena that your robot will be assessed in for this will be a square of 5.0 m x 5.0 m. There will only be wooden walls in the arena for this task, no other objects. The maze will be constructed to ensure there is always enough space for a TurtleBot3 Waffle to comfortably pass through any apertures or corridors.

1. To begin, the robot will be located in a blue *"Start Zone"* at the start of the maze.
1. The robot must then autonomously navigate the maze in order to find its way to a green *"Finish Zone"* (or get as close as possible to it).
1. The robot must do this without touching any of the arena walls: penalties will be applied for those that do (See [the table below](#marks)).
1. The robot's progress will be measured at 10% increments throughout the maze, using distance markers printed on the arena floor (where the finish line is at 100%).
1. The robot will have a maximum of 150 seconds to navigate the maze.

    <a name="launch"></a>

1. Your team's ROS package must contain a launch file called `task3.launch`, such that the functionality that you develop for this maze navigation task can be launched from your package via the command:
  
    ```bash
    roslaunch com2009_team{} task3.launch
    ```
  
1. Once again, this task will be assessed in simulation and the simulated environment (the arena containing the maze) will already be running before we attempt to execute your launch file.

## Simulation Resources

Within the `com2009_simulations` package there is an example maze navigation arena which can be used to develop and test out your team's maze navigation node(s) for this task. The simulation can be launched using the following `roslaunch` command:

```bash
roslaunch com2009_simulations maze_nav.launch
```

<center>
<figure markdown>
  ![](figures/maze_nav.jpg)
  <figcaption>The `maze_nav` arena for Task 3.</figcaption>
</figure>
</center>

!!! note
    1. The maze that will be used for the assessment **will be a different configuration to this**
    1. Corners in the final maze won't necessarily all be at right angles like they are in the simulation: some may be acute or obtuse!

## Marking {#marks}

There are **15 marks** available for this task in total, awarded based on the following criteria:

<center>

| Criteria | Marks | Details |
| :--- | :---: | :--- |
| **A**: Progress through the maze | 10/15 | Marks will be awarded based on the highest percentage value distance marker that your robot crosses within the 150-second time limit (it doesn't matter if the robot happens to turn around and move back behind a marker again at any point during the assessment). Marks will be awarded at 10% increments only (i.e. no fractional marks), but the **whole of the robot** must have crossed the progress marker in order to be awarded the associated marks. |
| **B**: An "incident-free-run" | 5/15 | If the robot completes the task (or the 150 seconds elapses) without it making contact with anything in the arena then your team will be awarded the maximum marks here. Marks will be deducted for each contact that the robot makes with the environment, to a minimum of 0 (i.e. there will be no negative marking here: the minimum mark that you can receive for this is zero). Your robot must *at least* pass the 10% progress marker to be eligible for these marks. |

</center>