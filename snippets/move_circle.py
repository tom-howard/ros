#!/usr/bin/env python3

import rospy # (1)!
# (2)!

class Circle():
    
    def __init__(self):
        self.node_name = # (3)!

        self.pub = rospy.Publisher() # (4)!
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = # (5)!
                
        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        # (6)!

        self.ctrl_c = True

    def main(self):
        while not self.ctrl_c:
            # (7)!
            
            
if __name__ == '__main__':
    # (8)!