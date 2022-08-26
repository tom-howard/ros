import rospy
from geometry_msgs.msg import Twist

node_name = "move_square"

movement = "fwd" # or "turn"
transition = True

rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(10) # hz

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel = Twist()

rospy.loginfo(f"The {node_name} node has been initialised...")
timestamp = rospy.get_time()
    
while not rospy.is_shutdown():
    elapsed_time = rospy.get_time() - timestamp
    if transition:
        timestamp = rospy.get_time()
        transition = False
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        print(f"Moving to state: {movement}")
    elif movement == "fwd":
        if elapsed_time > 5:
            movement = "turn"
            transition = True
        else:
            vel.linear.x = 0.1
            vel.angular.z = 0.0
    elif movement == "turn":
        if elapsed_time > 5:
            movement = "fwd"
            transition = True
        else:
            vel.angular.z = 0.2
            vel.linear.x = 0.0
    pub.publish(vel)
    rate.sleep()