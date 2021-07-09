# Booting up the Burger Bot

A quick guide to turning on the TurtleBot3 Burger robot! This tutorial provides instructions on turning on and connecting to the Burger for everyday use (not for the first time)! For instructions on how to connect to the Burger for the first time please see the [eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

Key Terms:
- **the Burger** - the physical robot, a Burger model of the TurtleBot3 series.
- **the Computer** - the main computer which will remotely be controlling the robot, also known as the Remote PC

*Disclaimer: While this tutorial may work with other TurtleBot3 robots, we do not guarantee this as this guide was written specifically for the Burger model robot. This tutorial will be using Ubuntu and ROS Kinetic and was writtin in 2021.*

## The General Process

In the terminal on the computer, type `ssh pi@<IP of Burger>` replacing <*IP of Burger*> with the IP of the Burger. 

Back at the Computer, in the terminal run `roscore` and wait for the process to finish. This will not indicate completion with another type prompt, roscore is considered fully launched when no more text is printed to the terminal. DO NOT PRESS ctrl+C OR CLOSE THIS TERMINAL. LEAVE THIS TERMINAL OPEN.

Finally, we'll bringup the Burger. Open a new terminal on the Computer and type `roslaunch turtlebot3_bringup turtlebot3_robot.launch`. Hit enter and the terminal will begin printing messages. The bringup process has completed once the line `[INFO] [a string of numbers]: Calibration End` has appeared. DO NOT PRESS ctrl+C OR CLOSE THIS TERMINAL. LEAVE THIS TERMINAL OPEN.

If there are no errors while running `roscore` or bringup, congratulations! You have now successfully connected the Burger and Computer to each other. If not, recheck the process or see the **Trouble Shooting** section below. To proceed and work on any projects, leave the current two (2) terminals as is or minimize the windows and open a third terminal for any further useage.


## Trouble Shooting

### Using a New Network
When using a new network, the IP address of the devices will change! Run though the following process to reconnect the Burger and Computer. Make sure both the Burger and the Computer are on the same network. 

Power on the Burger, monitor, and Computer. Connect the Burger to the monitor using the HDMI port on the Burger's Raspberry Pi. Plug in the keyboard and mouse into the Burger. Basically, the Burger will act as a system unit or CPU and the monitor should display a basic desktop.

Open a terminal (ctrl+alt+t) on the Computer and find the IP address using `ifconfig`. Write this down and repeat on the monitor to find the Burger's IP address.

Return to the Computer and in the terminal type `nano ~/.bashrc`, scroll down to the bottom. Update the IP addresses of the ROS_MASTER_URI and ROS_HOSTNAME to appear as below replacing <*IP of Computer*> and <*IP of Burger*> with the respective IP addresses:
```
export ROS_MASTER_URI=http://<IP of Computer>:11311
export ROS_HOSTNAME=<IP of Computer>
```
Press ctrl+x and then y to exit out of the bashrc file and save any changes. Type `source ~/.bashrc` to activate any changes made.
Repeat the nano-source process in the terminal on the monitor to match the following lines:
```
ROS_MASTER_URI = http://<IP of Computer>:11311
ROS_HOSTNAME = <IP of Burger>
```

Complete the General Process as decribed above.

### Error `alias not found` 
For when `nano ~/.bashrc` works but `source ~/.bashrc` throws an alias error.
Re-enter `nano ~/.bashrc` and check all lines to see if an alias line is spelled wrong. (e.g. `lias` instead of `alias`).
