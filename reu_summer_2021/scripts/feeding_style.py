#!/usr/bin/python

###
# Code closely modeled after ROS's turtlebot3_teleop_key file in turtlebot3_teleop package.
# To access this code type `rosed turtlebot3_teleop turtlebot3_teleop_key` into a terminal.
# TO CONTROL THE BURGER: moving straight is linear.x & turning is angular.z
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
    # linear: -0.22 to 0.22 where positive is forwards
    # angualr: -2.84 to 2.84 where positive is left

###
# Publisher and subscriber set up from wiki.ros.org/ROS/Tutorials/WritingPublisherSubsriber%28python%29
# Test if Publisher is publishing messages with `rostopic echo feeding -n 5` where -n 5 indicates echoing a maximum of 5 messages.
# Publisher written as global to allow move to be global.
###
rospy.init_node('computer', anonymous=True)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10) #10 Hz
move = Twist()

###
# To avoid repeated code. Stops all motor movement in Burger
###
def stop():
    move.linear.x = 0.00; move.linear.y = 0.00; move.linear.z = 0.0
    move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
    pub.publish(move)

###
# LINELINE PATHWAY 
# To access enter "lineline" when prompted for path style
###
def lineline(): 
    speed = userInput()
    if speed == "moderate" or "normal":
        print ("Running lineline normal")
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(11)
        stop()
    elif speed == "slow": # Burger moves straight forward slowly
        print("Running lineline slow")
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(55)
        stop()
    elif speed == "fast":
        print("Running lineline fast") # Burger moves straight forward quickly
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(5.5)
        stop()
    elif speed == "accelerate" or speed == "acc":
        print("Running lineline acceleration") # Burger moves straight forwards while accelerating (exaggerated)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(4)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(2)
        move.linear.x = 0.05; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(3)
        move.linear.x = 0.07; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(3)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.15; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1.75)
        stop()
    else:
        print("Invalid speed, restarting process.")

###
# CURVE PATHWAY
# To access enter "curve" when prompted for path style
###
def curve():
    speed = userInput()
    if speed == "moderate" or "normal":
        print("Running curve normal")
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(2)
        move.linear.x = 0.06; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -1
        pub.publish(move)
        rospy.sleep(0.5)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(2)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.2
        pub.publish(move)
        rospy.sleep(7.5)
        stop()

###
# SINELINE PATHWAY 
# To access enter "sineline" when prompted for path style
###
def sineline(): 
    speed = userInput()
    if speed == "slow":
        print("Running sineline slow") # 2 forward, 5r, 10l, 5r, continue straight
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(5)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.05
        pub.publish(move)
        rospy.sleep(10)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -0.05
        pub.publish(move)
        rospy.sleep(20)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.05
        pub.publish(move)
        rospy.sleep(10)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(13)
        stop()
    elif speed == "fast": # unknown
        print("Running sineline fast")
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.2)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 1.5
        pub.publish(move)
        rospy.sleep(0.5)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.5)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -1.5
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 1.5
        pub.publish(move)
        rospy.sleep(1.3)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.2)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -1.5
        pub.publish(move)
        rospy.sleep(0.75)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(2.5)
        stop()
    elif speed == "accelerate" or speed == "acc":
        print("Running slineline acceleration") # 2 forward, 3r, 6l, 6r, 3l, accelerate to 20
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(5)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.17
        pub.publish(move)
        rospy.sleep(3)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -0.17
        pub.publish(move)
        rospy.sleep(6)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.17
        pub.publish(move)
        rospy.sleep(3)
        # accelerate forwards for remaining straight line
        move.linear.x = 0.09; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(2)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.15; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        stop()
    else:
        print("Invalid speed, restarting process.")

###
# SINESINE PATHWAY 
# To access enter "sinesine" when prompted for path style
###
def sinesine():
    speed = userInput()
    if speed == "moderate" or "normal":
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.5
        pub.publish(move)
        rospy.sleep(2.2)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -0.5
        pub.publish(move)
        rospy.sleep(3)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(4.5)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.5
        pub.publish(move)
        rospy.sleep(1.5)
        stop()
    elif speed == "slow":
        print("Running sinesine slow") # 3 forward, 4r, 6l, 6r, 4l
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(10) # exit curtain
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.2
        pub.publish(move)
        rospy.sleep(5)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(3)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -0.2
        pub.publish(move)
        rospy.sleep(10)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(10)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.2
        pub.publish(move)
        rospy.sleep(10)
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -0.2
        pub.publish(move)
        rospy.sleep(5)
        stop()
    elif speed == "fast":
        print("Running sinesine fast") # unknown
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 1.5
        pub.publish(move)
        rospy.sleep(0.75)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.75)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -1.5
        pub.publish(move)
        rospy.sleep(1.5)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1.7)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 1.5
        pub.publish(move)
        rospy.sleep(1.15)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1.5)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -1.5
        pub.publish(move)
        rospy.sleep(0.4)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(0.5)
        stop()
    elif speed == "accelerate" or speed == "acc":
        print("Running slinesine acceleration") # 3 forwards, 4r, 6l, 3 forward, 10r, 3l
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(5) # exit curtain
        move.linear.x = 0.02; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.2
        pub.publish(move)
        rospy.sleep(5)
        move.linear.x = 0.03; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(3.5)
        move.linear.x = 0.05; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -0.5
        pub.publish(move)
        rospy.sleep(4.5)
        move.linear.x = 0.1; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(1.25)
        move.linear.x = 0.15; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 1.5
        pub.publish(move)
        rospy.sleep(4)
        move.linear.x = 0.18; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
        rospy.sleep(2.2)
        move.linear.x = 0.2; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = -2
        pub.publish(move)
        rospy.sleep(0.5)
        stop()
    else:
        print("Invalid speed, restarting process.")

###
# Retrieves user input on the speed of the robot. Created to avoid repeated code and for clarity.
# Returns speed (string).
###
def userInput():
    speed = raw_input("Enter speed: ").lower()
    return (speed)

def run():
    while not rospy.is_shutdown(): #while code is still running without interrupt (ctrl+d)
        path = raw_input("Enter path style: ").lower() #get user input on path, ignore capitalization
        #depending on path input choose appropriate path function
        if path == "lineline":
            lineline()
        elif path == "sineline":
            sineline()
        elif path == "sinesine":
            sinesine()
        elif path == "curve":
            curve()
        elif path == "stop": 
            # to stop the robot while its moving, terminate the current session and relaunch
            # when prompted for path style, type "stop"
            stop()
        elif path == "recall":
            print("recalling.")
            move.linear.x = -0.1; move.linear.y = 0.0; move.linear.z = 0.0
            move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
            pub.publish(move)
            rospy.sleep(11.5)
            stop()
        else:
            print("Invalid path choice. Please choose again. To terminate the code press ctrl+c or ctrl+d.")

###
# Main body function. Calls run() unless interrupt (ctrl+c)
###
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
