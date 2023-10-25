import rospy
from geometry_msgs.msg import Twist # (1)!
from nav_msgs.msg import Odometry # (2)!
from tf.transformations import euler_from_quaternion # (3)!
from math import sqrt, pow, pi # (4)!

class Square():
    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose # (5)!
        position = pose.position
        orientation = pose.orientation

        pos_x = position.x # (6)!
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        ) # (7)!

        self.x = pos_x # (8)!
        self.y = pos_y
        self.theta_z = yaw

        if not self.first_message: # (9)!
            self.first_message = True

    def __init__(self):
        node_name = "move_square"
        self.first_message = False
        self.turn = False # (10)!

        # (11)!
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # (12)!
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.vel = Twist() # (13)!

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist()) # (14)!
        self.ctrl_c = True

    def main(self):
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot move in a square of
            # dimensions 1 x 1m...

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            self.rate.sleep() # maintain the loop rate @ 10 hz

if __name__ == "__main__":
    node = Square()
    node.main()
    