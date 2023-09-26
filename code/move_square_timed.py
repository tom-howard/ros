import rospy # (1)!
from geometry_msgs.msg import Twist # (2)!

movement = "state1" # "state2, state3 etc..."
transition = True

rospy.init_node("move_waffle", anonymous=True) # (3)!
rate = rospy.Rate(10) # (4)!

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # (5)!
vel = Twist() # (6)!

rospy.loginfo(f"The node has been initialised...")
timestamp = rospy.get_time() # (7)!
    
while not rospy.is_shutdown(): # (8)!
    elapsed_time = rospy.get_time() - timestamp # (9)!
    if transition: # (10)!
        timestamp = rospy.get_time()
        transition = False
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        print(f"Moving to state: {movement}")
    elif movement == "state1": # (11)!
        if elapsed_time > 2:
            movement = "state2"
            transition = True
        else:
            vel.linear.x = 0.05
            vel.angular.z = 0.0
    elif movement == "state2": # (12)!
        if elapsed_time > 4:
            movement = "state1"
            transition = True
        else:
            vel.angular.z = 0.2
            vel.linear.x = 0.0
    pub.publish(vel) # (13)!
    rate.sleep() # (14)!