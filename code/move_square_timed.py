import rospy
from geometry_msgs.msg import Twist

node_name = "move_waffle"

movement = "state1" # "state2, state3 etc..."
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
    elif movement == "state1":
        if elapsed_time > 2:
            movement = "state2"
            transition = True
        else:
            vel.linear.x = 0.05
            vel.angular.z = 0.0
    elif movement == "state2":
        if elapsed_time > 4:
            movement = "state1"
            transition = True
        else:
            vel.angular.z = 0.2
            vel.linear.x = 0.0
    pub.publish(vel)
    rate.sleep()