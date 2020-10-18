#!/usr/bin/env python

from __future__ import print_function

# Load ROS
import roslib; roslib.load_manifest('gtddp_drone')
import rospy

# ROS Messages
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

# Keyboard Imports
import sys, select, termios, tty

# Color definitions
class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'

# Key combos that should exit the program
exitKeys = ['\x03', '\x1A', 'q'] # Ctrl-C, Ctrl-Z

# Get the key pressed by the user
def getKey(settings):
    tty.setcbreak(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def manual_control(key, reset_pub, takeoff_pub, manual_ctrl_pub):
    # R is for reset. Reset a crashed drone without a power cycle
    global vel

    # Reset vel cmd
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.z = 0

    if (key == 'r'):
        reset_msg = Empty()
        reset_pub.publish(reset_msg)
    elif (key == 't'):
        takeoff_msg = Empty()
        takeoff_pub.publish(takeoff_msg)
    # W commands a positive vertical speed
    elif (key == 'a'):
        vel.linear.z = SPEED
        manual_ctrl_pub.publish(vel)
    # S commands a negative vertical speed
    elif (key == 'z'):
        vel.linear.z = -SPEED
        manual_ctrl_pub.publish(vel)
    # C commands a positive pitch
    elif (key == 'c'):
        vel.linear.x = SPEED
        manual_ctrl_pub.publish(vel)
    # X commands a negative pitch
    elif (key == 'x'):
        vel.linear.x = -SPEED
        manual_ctrl_pub.publish(vel)
    # E commands a positive roll
    elif (key == 'e'):
        vel.linear.y = SPEED
        manual_ctrl_pub.publish(vel)
    # D commands a negative roll
    elif (key == 'd'):
        vel.linear.y = -SPEED
        manual_ctrl_pub.publish(vel)
    # Turn Left
    elif (key == 'j'):
        vel.angular.z = SPEED
        manual_ctrl_pub.publish(vel)
    # Turn Right
    elif (key == 'k'):
        vel.angular.z = -SPEED
        manual_ctrl_pub.publish(vel)


vel = Twist()
SPEED = 0.3

# Main Function
if __name__=="__main__":
    # Enable Termios
    settings = termios.tcgetattr(sys.stdin)

    # Set up ROS publishers
    # Drone 1
    takeoff_pub = rospy.Publisher('/drone1/ardrone/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/drone1/ardrone/land', Empty, queue_size=10)       # emergency land the drone
    reset_pub = rospy.Publisher('/drone1/ardrone/reset', Empty, queue_size=10)     # Reset a crashed drone
    manual_ctrl_pub = rospy.Publisher('/drone1/cmd_vel', Twist, queue_size=10)     # Manual control

    # Drone 2
    takeoff_pub_2 = rospy.Publisher('/drone2/ardrone/takeoff', Empty, queue_size=10)
    land_pub_2 = rospy.Publisher('/drone2/ardrone/land', Empty, queue_size=10)       # emergency land the drone
    reset_pub_2 = rospy.Publisher('/drone2/ardrone/reset', Empty, queue_size=10)     # Reset a crashed drone
    manual_ctrl_pub_2 = rospy.Publisher('/drone2/cmd_vel', Twist, queue_size=10)     # Manual control

    # Both Drones
    init_pub = rospy.Publisher('/gtddp_drone/start', Empty, queue_size=10)  # GO button

    # Selected drone
    active_drone = 1

    # Initialize the ROS node
    rospy.init_node('keyboard_node')

    # Start the main loop
    while(1):
        # Get a key press from the user
        try:
            key = getKey(settings)
        # If there is an error, print it
        except Exception as e:
            print(e)
        # Restore terminal settings
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # print(key)
        if(key == '1'):
            active_drone = 1
        elif(key == '2'):
            active_drone = 2
        # If the key is the E-Stop key, land the drones
        elif (key == ' '):
            land_msg = Empty()
            land_pub.publish(land_msg)
            land_pub_2.publish(land_msg)
        # G is for go. Initialize the optimizer
        elif (key == 'g'):
            init_msg = Empty()
            init_pub.publish(init_msg)
        # If the key is one of the exit keys, close the program
        elif (key in exitKeys):
            break

        # Control each drone individually based on which one is active
        if active_drone == 1:
            manual_control(key, reset_pub, takeoff_pub, manual_ctrl_pub)
        elif active_drone == 2:
            manual_control(key, reset_pub_2, takeoff_pub_2, manual_ctrl_pub_2)

        key = ""
