#!/usr/bin/python

###
# Code closely modeled after ROS's turtlebot3_teleop_key file in turtlebot3_teleop package.
# To access this code type `rosed turtlebot3_teleop turtlebot3_teleop_key` into a terminal.
###

#Imports for function
import roslib
import rospy, time
from math import sin,cos,atan2,sqrt
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16, String

# Global variables just for information
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
# Allowable motor values are
    # linear: -0.22 to 0.22
    # angualr: -2.84 to 2.84

###
# Publisher and subscriber set up from wiki.ros.org/ROS/Tutorials/WritingPublisherSubsriber%28python%29
# Test if Publisher is publishing messages with `rostopic echo feeding -n 5` where -n 5 indicates echoing a maximum of 5 messages.
# Publisher written as global to allow move to be global.
###
rospy.init_node('computer', anonymous=True)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10) #10 Hz
move = Twist()

#TO CONTROL THE BURGER: moving straight is linear.x & turning is angular.z

###
# To avoid repeated code. Stops all motor movement in Burger
###
def stop():
    move.linear.x = 0.00; move.linear.y = 0.00; move.linear.z = 0.0
    move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
    pub.publish(move)

###
# line line pathway option
# For now, only working with lineline() function to get prublisher subscriber working, ignore contents of other functions
###
def lineline(): 
    speed = userInput()
    if speed == "slow": # Burger moves straight forward for 40 seconds
        print("Running lineline slow")
        move.linear.x = 0.05; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(40)
        stop()
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
        path = raw_input("Enter path style: ").lower() #get user input on path

        #depending on path input choose appropriate path
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
