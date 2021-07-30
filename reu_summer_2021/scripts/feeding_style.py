#!/usr/bin/python

#Imports for function
import roslib; # roslib.load_manifest("chairbot_neato_node")
import rospy, time
from math import sin,cos,atan2,sqrt
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16, String

###
# Publisher and subscriber set up from wiki.ros.org/ROS/Tutorials/WritingPublisherSubsriber%28python%29
# Establish pub as a global variable to publish to the topic "feeding"
# Test if Publisher is publishing messages with `rostopic echo feeding -n 5` where -n 5 indicates echoing a maximum of 5 messages.
###
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.init_node('computer', anonymous=True)
rate = rospy.Rate(10) #10 Hz
move = Twist()
#pub = rospy.Publisher("feeding", String, queue_size=10)
#rospy.init_node('computer', anonymous=True)
#rate = rospy.Rate(10) #10 Hz

###
# line line pathway option
# For now, only working with lineline() function to get prublisher subscriber working, ignore contents of other functions
###
def lineline(): 
    speed = userInput()
    if speed == "slow":
        print("Lineline slow")
        move.linear.x = 0.5
        pub.publish(move)
        rospy.sleep(10)
        move.linear.x=0
        pub.publish(move)
        print("done")
    elif speed == "fast":
        print("Lineline fast")
        pub.publish("Lineline fast")
    elif speed == "accelerate" or speed == "acc":
        #line line acceleration, how to calculate?
        print("Lineline acceleration")
        pub.publish("Lineline acceleration")
    else:
        print("Invalid speed, restarting process.")

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
# Retrieves user input on the speed of the robot. Created to avoid repeated code and for clarity.
# Returns speed (string).
###
def userInput():
    speed = raw_input("Enter speed: ").lower()
    return (speed)

def run():
    #while code is still running without interrupt
    while not rospy.is_shutdown():
        while True:
            #get user input on path
            path = raw_input("Enter path style: ").lower()

            if path == "lineline":
                lineline()
            elif path == "sineline":
                sineline()
            elif path == "sinesine":
                sinesine()
            else:
                print("Invalid path choice. Please choose again. To terminate the code press ctrl+c or ctrl+d.")

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass