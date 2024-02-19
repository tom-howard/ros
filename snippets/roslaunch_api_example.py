#!/usr/bin/env python3

import roslaunch
import rospy

file_path = "/full/path/to/text/file"

rospy.init_node("pointless_file_generator_caller", anonymous=True)
rate = rospy.Rate(0.5)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

print(f"Saving file at time: {rospy.get_time()}...")
node = roslaunch.core.Node(
    package="tuos_examples",
    node_type="pointless_file_generator",
    args=f"-f {file_path}"
)
process = launch.launch(node)
rate.sleep()
