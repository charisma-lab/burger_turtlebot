#!/usr/bin/python

#Imports for function
import roslib; # roslib.load_manifest("chairbot_neato_node")
import rospy, time
from math import sin,cos,atan2,sqrt
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16, String
#from sensor_msgs.msg import Joy
#from std_msgs.msg import Int8

def burger():
    rospy.init_node('burger', anonymous=True)
    rospy.Subscriber("feeding", String, queue_size=10)

    rospy.spin()

if __name__=='_main_':
    burger()