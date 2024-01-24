#!/usr/bin/env python3

import roslaunch
import rospy

map_path = "path/to/my_map"

rospy.init_node("map_getter", anonymous=True)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

print(f"Saving map at time: {rospy.get_time()}...")
node = roslaunch.core.Node(package="map_server",
                           node_type="map_saver",
                           args=f"-f {map_path}")
process = launch.launch(node)
