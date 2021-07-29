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

###
# Publisher and subscriber set up from wiki.ros.org/ROS/Tutorials/WritingPublisherSubsriber%28python%29
###
def __init__(self):
    rospy.Publisher("feeding", String, queue_size=10)
    rospy.init_node('computer', anonymous=True)
    rate = rospy.Rate(10) #10 Hz

###
# line line pathway option
###
def lineline(self): 
    speed, timer = userInput()
    if speed == "slow":
        self._robot.setMotors(0, 200, 50)
        sleep(20)
    elif speed == "fast":
        self._robot.setMotors(0, 200, 200)
        sleep(5)
    else:
        #line line acceleration, how to calculate?
        self._robot.setMotors(0, 200, speed)
        sleep(timer)

###
# sine line pathway option
# still need to calculate pathway, look at 'discarded lit search papers' for more
###
def sineline(self): 
    speed, timer = userInput()
    if speed == "slow":
        self._robot.setMotors(0, 200, 50)
        sleep(timer)
    elif speed == "fast":
        self._robot.setMotors(0, 200, 200)
        sleep(timer)
    else:
         # line line acceleration, how to calculate?
        self._robot.setMotors(0, 200, speed)
        sleep(timer)

###
# sine sine pathway option
# still need to calculate pathway, look at 'discarded lit search papers' for more
###
def sinesine(self):
    speed, timer = userInput()
    if speed == "slow":
        self._robot.setMotors(0, 200, 50)
        sleep(timer)
    elif speed == "fast":
        self._robot.setMotors(0, 200, 200)
        sleep(timer)
    else:
        # line line acceleration, how to calculate?
        self._robot.setMotors(0, 200, speed)
        sleep(timer)

###
# Retrieves user input on the speed of the robot.
# Returns speed (string) and timer (int)
###
def userInput():
    #get user input on speed
    speed = raw_input("Enter speed: ")
    #create timers for each speed
    if speed=="slow":
        timer=50
    elif speed=="fast":
        timer=10
    else:
        timer=30
    return (speed, timer)

def run():
    #while code is still running without interrupt
    while not rospy.is_shutdown():
        while True:
            #get user input on path
            path = raw_input("Enter path style: ")

            robot = BurgerNode()
            if path == "lineline":
                robot.lineline()
            elif path == "sineline":
                robot.sineline()
            elif path == "sinesine":
                robot.sinesine()
            else:
                print("Invalid path choice. Please choose again. To terminate the code press ctrl+c.")

if __name__ == '__main__':
    try:
        run()
        except rospy.ROSInterruptException:
            pass
    