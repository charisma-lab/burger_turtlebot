#!/usr/bin/env python3

#Imports for function
#import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time
from math import sin,cos,atan2,sqrt
from std_msgs.msg import String
#from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped
#from std_msgs.msg import UInt16, String
#from sensor_msgs.msg import Joy
#from std_msgs.msg import Int8
#export ROS_PACKAGE_PATH=/home/user/ros/ros-pkg:/

###
# Function to run the lineline (straight) path.
# Need to figure out how to time path to execute.
###
def lineline(self):
    speed, timer = userInput()
    if speed == "slow":
        self._robot.setMotors(0, 200, 50)
        rospy.sleep(20)
    elif speed == "fast":
        self._robot.setMotors(0, 200, 200)
        rospy.sleep(5)
    else:
        #line line acceleration, how to calculate?
        self._robot.setMotors(0, 200, speed)
        rospy.sleep(timer)

###
# Function to run the sineline (wave then straight) path.
# Need to figure out how to calculate path to execute.
###
def sineline(self):
    speed, timer = userInput()
    if speed == "slow":
        self._robot.setMotors(0, 200, 50)
        rospy.sleep(timer)
    elif speed == "fast":
        self._robot.setMotors(0, 200, 200)
        rospy.sleep(timer)
    else:
        # line line acceleration, how to calculate?
        self._robot.setMotors(0, 200, speed)
        rospy.sleep(timer)

###
# Function to run the sinesine (wavy) path.
# Need to figure out how to calculate path to execute.
###
def sinesine(self):
    speed, timer = userInput()
    if speed == "slow":
        self._robot.setMotors(0, 200, 50)
        rospy.sleep(timer)
    elif speed == "fast":
        self._robot.setMotors(0, 200, 200)
        rospy.sleep(timer)
    else:
        # line line acceleration, how to calculate?
        self._robot.setMotors(0, 200, speed)
        rospy.sleep(timer)

###
# Repeated code. Retrieves user input on speed and selects a timer based on that. 
# Returns speed (string) and timer (int).
###
def userInput():
    #get user input on speed
        speed = input("Enter speed: ")
    #create timers for each speed
        if speed=="slow":
            timer=50
        elif speed=="fast":
            timer=10
        else:
            timer=30
        return (speed, timer)

if __name__ == '__main__':
    #publish to Burger using /cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('feeding', anonymous=True)

    while not rospy.is_shutdown(): #while the program is still running
        while True:
            #get user input on path
            path = input("Enter path style: ")
            #pick path according to user input
            if path == "lineline":
                robot.lineline()
            elif path == "sineline":
                robot.sineline()
            elif path == "sinesine":
                robot.sinesine()
            else:
                print("Invalid path choice. Please choose again. To terminate the code press ctrl+c.")