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

def callback(msg):
    print msg.data


rospy.init_node('burger', anonymous=True)
sub = rospy.Subscriber("feeding", String, callback, queue_size=10)
rospy.spin()