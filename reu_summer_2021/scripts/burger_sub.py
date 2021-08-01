#!/usr/bin/python

###
# PLEASE IGNORE THE CONTENTS OF THIS SCRIPT!!
#  When using the topic cmd_vel it is not necessary to write a subscriber. Therefore, the code in 
#  this script is extreneous. The topic has been updated for consistency and the script will remain 
#  in the repo until final clean up, however no other edits will be made to burger_sub.py until 
#  further notice.
###

#Imports for function
import roslib; # roslib.load_manifest("chairbot_neato_node")
import rospy, time
from math import sin,cos,atan2,sqrt
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16, String
#from sensor_msgs.msg import Joy
#from std_msgs.msg import Int8

# Global variable speed
speed = "temp"

###
# callback retrives the message from the publisher and assigns its value to the variable speed
# Based on the value of speed, callback calls the appropriate function.
###
def callback(msg):
    speed = msg.data.lower()
    print(speed)
    if speed.startswith("lineline"):
        rospy.loginfo("Inside lineline")
    elif speed.startswith("sineline"):
        rospy.loginfo("Inside sineline")
    elif speed.startswith("sinesine"):
        rospy.loginfo("Inside sinesine")
    else:
        pass

###
# Define subscriber node.
###
rospy.init_node('burger', anonymous=True)
sub = rospy.Subscriber("cmd_vel", String, callback, queue_size=10)
rospy.spin()