# Booting up the Burger Bot

A quick guide to turning on the TurtleBot3 Burger robot! This tutorial provides instructions on turning on and connecting to the Burger for everyday use (not for the first time)! For instructions on how to connect to the Burger for the first time please see the [eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

Key Terms:
- **the Burger** - the physical robot, a Burger model of the TurtleBot3 series.
- **the Computer** - the main computer which will remotely be controlling the robot, also known as the Remote PC

Additionally, the **Trouble Shooting, Mini Guides, and More** section below can be used to help resolve some issues we have run into as well as how to develop new packages and other starter concerns.

*Disclaimer: While this tutorial may work with other TurtleBot3 robots, we do not guarantee this as this guide was written specifically for the TurtleBot3 Burger model of robot. This tutorial will be using Ubuntu/Linux, ROS Kinetic, GitHub, Python2.7, and VSCode and was writtin in 2021.*

## The General Process

Turn both the computer and Burger on. Make sure the Burger is charged ahead of time. If the Burger begins beeping, it is likely that the battery is low and the Burger will need to be recharged.

In the terminal run `roscore` and wait for the process to finish. This will not indicate completion with another type prompt, roscore is considered fully launched when no more text is printed to the terminal. DO NOT PRESS ctrl+C OR CLOSE THIS TERMINAL. LEAVE THIS TERMINAL OPEN. 

Remaining on the Computer, open a new terminal (ctrl+alt+t) and type `ssh pi@<IP of Burger>` replacing <*IP of Burger*> with the IP of the Burger.

Finally, bringup the Burger. Open a new terminal on the Computer and type `roslaunch turtlebot3_bringup turtlebot3_robot.launch`. Hit enter and the terminal will begin printing text. The bringup process has completed once the line `[INFO] [a string of numbers]: Calibration End` has appeared (where "a string of numbers" is a values such as ##########.######). DO NOT PRESS ctrl+C OR CLOSE THIS TERMINAL. LEAVE THIS TERMINAL OPEN.

If there are no errors while running `roscore` or bringup, congratulations! You have now successfully connected the Burger and Computer to each other. If not, recheck the process or see the **Trouble Shooting** section below. To proceed and work on any projects, leave the current two (2) terminals as is or minimize the windows and open a third terminal for any further useage.


## Trouble Shooting, Mini Guides, and More

### Controlling Burger with its Motors
Sometimes hard coding a path is easier and more consistant than manually manipulating the robot with ROS's premade teleop packages. Here's how to control the Burger's motors through code.
```
#imports
import rospy
from geometry_msgs.msg import Twist

###
#Establish a publisher on the cmd_vel topic
# <node_name> is the name of your node, this can be anything within reason
# move is the name of our Twist variable, you can name this anything within reason and this 
#  variable is what gets published to the Burger
###
rospy.init_node('<node_name>', anonymous=True)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10) #10 Hz
move = Twist()

if __name__ == '__main__':
    try:
        #Controlling the motors of Burger is done specifically with the following two lines:
        move.linear.x = 0.05; move.linear.y = 0.0; move.linear.z = 0.0
        move.angular.x = 0.0; move.angular.y = 0.0; move.angular.z = 0.0
        pub.publish(move)
    except rospy.ROSInterruptException:
        pass
```
To make the Burger move forward (or backward) linearly, change the value of `move.linear.x`, to make the Burger turn, change the value of `move.angular.z`. The allowable motor values are as follows:
* Linear: -0.22 to 0.22
* Angular: -2.84 to 2.84

Any values out of bounds will result in no movement. To have the motors run at a specific velocity for only a set duration of time, use `rospy.sleep(<#>)` after publishing \(where \<#> is the number of seconds for the desired duration). To stop all motor movement, set all six (6) linear and angular variables to `0.0`. Remember to run `pub.publish(move)` after every velocity change for the change to take affect in the Burger.

### Creating a Basic Launch File
It is recommended to build a launch file to make running packages easier. A launch file starts up all specified nodes with one command `roslaunch <package_name> <file.launch>` where <package_name> is replaced with the name of the desired package and <file.launch> is a .launch file.
1. Enter your package using `roscd <package_name>` or cd commands.
2. Make a launch folder for launch files (you can have multiple launch files for one package). Use `mkdir launch` then `cd launch` to enter the folder.
3. Open the Files app and navigate to the launch folder created in step 2. The path may be similar to *catkin_ws/src/<GitHub_repo>/<package_name>*.
4. Right click anywhere in the empty space in the Files app, scroll over "New Document", and select "Empty Document". A second window should open up which looks like a text editor.
5. Return to Files and right click on the new Untitled Document. Rename the document and end the name with ".launch" (i.e. package.launch).
6. If the document is not already open, double click on the document to open it.
7. In the .launch file write
```
<launch>
    <node pkg="<package_name>" name="<name>" type="<file.py>" cwd="node" />
</launch>
```
Where \<name> can be anything and <file.py> is the full name of the file where the node is defined (including the file extension). Include a definition line for each node which needs to be launched. To have a node print to the terminal, add `output = "screen"` to the node's launch line. For more details about the contents of a .launch file as well as defining parameters, see [the wiki page](http://wiki.ros.org/roslaunch/XML) and [this page](https://answers.ros.org/question/197522/introduction-on-writing-launch-files/).

### Error `alias not found` 
For when `nano ~/.bashrc` works but `source ~/.bashrc` throws an alias error.
Re-enter `nano ~/.bashrc` and check all lines in the bashrc file to see if an alias line is spelled wrong. (e.g. `lias` instead of `alias`).

### Establishing Publisher and Subscriber Nodes
Please see pages 32-38 of [Programming Robots with ROS](https://drive.google.com/file/d/1mgKi8iFgh-3NIF7gad5OK-ws1ZvxGYp3/view). Examples 3-1 and 3-2 are images of the basic set up which can be easily manipulated. Remember to `pub.publish(<message>)` for every \<message> sent from the publisher to the subscriber.

### Finding an IP Address
Every device has a unique IP address when it connects to a network. This address will change between different networks. To check the IP address of a device follow these steps:

Open a terminal on the device (ctrl+alt+t). Type `ifconfig`. Towards the bottom of the text printed to the terminal will be a line starting with "inet addr:", the string of numbers afterwards is the IP address of the device.

### Making a Package
This tutorial will closely follow the tutorial from [the ROS website](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) but with less explanation and combined with [this](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) article on setting up publisher and subcriber nodes.
1. Assuming a catkin_ws workspace is already set up, enter your source file using `cd ~/catkin_ws/src`.
2. Create the package using `catkin_create_pkg <package_name> [depend1] [depend2]`. As an example, to create a package with the name "beginner_tutorials" and the dependencies std_msgs, rospy, and roscpp would look like `catkin_create_pkg beginner_tutorials std_msgs rospy roscpp`.
3. Back out to the workspace with `cd ~/catkin_ws`.
4. Run `catkin_make`.
5. Run `. ~/catkin_ws/devel/setup.bash` to source the generated setup file.
6. \[OPTIONAL STEP\] Set up your package.xml with a description, maintainer tag, etc. To update these enter the package.xml file in your package and alter any necessary tags.  At the very least we recommend updating the description and author tags. An update to the maintainer tag would look like `<maintainer email="you@yourdomain.tld">Your Name</maintainer>` to `<maintainer email="charisma@beanloaf.beans">Charisma Labs</maintainer>`.
7. Run `source /opt/ros/kinetic/setup.bash`. If you are using a version other than kinetic, replace "kinetic" with your version.
8. Backout into your catking_ws using `cd ~/catkin_ws` or `cd ..` until in catkin_ws.
9. Run `catkin_make <package_name>` replacing <package_name> with your package's name.
10. Enter the package with `cd ~/catkin_ws/src/<package_name>`.
11. Create a 'scripts' folder by running `mkdir scripts` and then enter the folder with `cd scripts`.
12. Create a file (in our case a .py file) inside of the scripts folder from step 11. This can be done by opening a code editor (i.e. pycharm or vscode), using the directory to enter into the scripts folder in your new package, and then creating a new file in your choice of language.
13. Run `source catkin_ws/devel/setup.bash`.
14. Run `chmod +x <.py_file_name>`.
15. Run `ls -la` to verify that the .py file is now executable. The file should have an x in the discriptor on the same line as the file name. 
16. To run the .py file run `rosrun <package_name> <.py_file_name>`.

### Predefining the Turtlebot3's Model
Whenever launching a package, turtlebot3 requires the user to first define the model of robot they are using with `export TURTLEBOT3_MODEL=${TB3_MODEL}` where ${TB3_MODEL} is replaced with the robot's model (burger, waffle, or waffle_pi). In order to skip this step the user can predefine the turtlebot3's model.

First, connect to the Burger as shown above. Then type `echo 'export TURTLEBOT3_MODEL=${TB3_MODEL}' >> ~/.bashrc` into the terminal replacing ${TB3_MODEL} with the appropriate model (in our case we use `echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc`) and hit enter. Next, type `source ~/.bashrc` to activate the changes made. Now the model has been predefined.

### Pushing a Developed Package to a Premade GitHub Repository
To push an existing project to a new repository on GitHub, please follow [these](https://docs.github.com/en/github/importing-your-projects-to-github/importing-source-code-to-github/adding-an-existing-project-to-github-using-the-command-line) instructions. The instructions below are to push an existing project to an already made repo.
1. Open the repo on github.com.
2. Click on the green button to the top right of the files list that says "code" and select the clipboard icon next to the https link to copy the link.
3. Open a terminal (ctrl+alt+t).
4. Change the directory into where you wish to clone the repo. We suggest using `cd catkin_ws`, then `cd src` and cloning here.
5. Clone the repository onto the computer with `git clone <repo_link>` replacing <repo_link> with the link copied from the step above.
6. Find the package you wish to add to the repo, then drag and drop it into the the desired location inside of the repo which was cloned in the step above.
7. Backout to the root of your package (if you are working in catkin_ws the command to use is `cd catkin_ws`).
8. Recompile your package with `catkin_make <package_name>`.
9. Enter the repo using `roscd <repo_name>` or cd commands.
10. Run `git status`, the name of your package should appear in red text.
11. Run `git add *` or `git add <package_name>` to add your package to the repo. Note: the command with '\*' will add all files not currently in the repo while the command with the package name will only add the specified package or file.
12. Run `git status` to verify your package has been added to the repo.
13. Run `git commit -m "<commit_message>"` replacing <commit_message> with a description of what you are committing to the repo.
14. Run `git push`. 
To continue working on the project and updating to GitHub, repeat steps 9 through 14 in order to push new edits. Before starting every work session be sure to `git pull` to be working on the most up-to-date files. 

### Unable to Contact My Own Server or 100% Packet Loss
For when `ping <IP>` or `roscore` fails, check to see if the IPs of the devices are correct.

In a terminal on the Computer, find the IP address with `ifconfig`. Staying on the Computer, open a second terminal and type `nano ~/.bashrc`, scroll down and check if the master URI and host name IPs' match the Computer's current IP. Press ctrl+x and save if any changes were made. Next, access a terminal on the monitor and find the Burger's IP address. Compare it to the IP address in `nano ~/.bashrc`. Reference [Using A New Network](https://github.com/charisma-lab/burger_turtlebot/blob/main/Booting%20up%20the%20Burger%20Bot.md#using-a-new-network) above for the correct placement of each IP address. Exit the bashrc file and save any changes.

### Using a New Network
When using a new network, the IP address of the devices will change! Run though the following process to reconnect the Burger and Computer. Make sure both the Burger and the Computer are on the same network. 

Power on the Burger, monitor, and Computer. Connect the Burger to the monitor using the HDMI port on the Burger's Raspberry Pi. Plug in the keyboard and mouse into the Burger. The Burger will act as a system unit or CPU and the monitor should display a basic desktop.

Open a terminal (ctrl+alt+t) on the Computer and find the IP address of the Computer. Write this down and repeat on the monitor to find the Burger's IP address.

Return to the Computer and in the terminal type `nano ~/.bashrc`, scroll down to the bottom. Update the IP addresses of the ROS_MASTER_URI and ROS_HOSTNAME to appear as below replacing <*IP of Computer*> and <*IP of Burger*> with the respective IP addresses:
```
export ROS_MASTER_URI=http://<IP of Computer>:11311
export ROS_HOSTNAME=<IP of Computer>
```
Press ctrl+x and and accept any changes to exit out of the bashrc file and save any changes. Type `source ~/.bashrc` to activate any changes made.
Repeat the nano/source process in the terminal on the monitor to match the following lines:
```
export ROS_MASTER_URI=http://<IP of Computer>:11311
export ROS_HOSTNAME=<IP of Burger>
```

To finish, complete the General Process as decribed above.
