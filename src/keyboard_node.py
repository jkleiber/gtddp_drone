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


vel = Twist()
SPEED = 0.2

# Main Function
if __name__=="__main__":
    # Enable Termios
    settings = termios.tcgetattr(sys.stdin)

    # Set up ROS publishers
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=10)       # emergency land the drone
    reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=10)     # Reset a crashed drone
    init_pub = rospy.Publisher('/gtddp_drone/start', Empty, queue_size=10)  # GO button
    manual_ctrl_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)     # Manual control

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
        # If the key is the E-Stop key, land the drone
        if (key == ' '):
            land_msg = Empty()
            land_pub.publish(land_msg)
        # G is for go. Initialize the optimizer
        elif (key == 'g'):
            init_msg = Empty()
            init_pub.publish(init_msg)
        # R is for reset. Reset a crashed drone without a power cycle
        elif (key == 'r'):
            reset_msg = Empty()
            reset_pub.publish(reset_msg)
        elif (key == 't'):
            takeoff_msg = Empty()
            takeoff_pub.publish(takeoff_msg)
        # W commands a positive vertical speed
        elif (key == 'a'):
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = SPEED
            manual_ctrl_pub.publish(vel)
        # S commands a negative vertical speed
        elif (key == 'z'):
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = -SPEED
            manual_ctrl_pub.publish(vel)
        # C commands a positive pitch
        elif (key == 'c'):
            vel.linear.x = SPEED
            vel.linear.y = 0
            vel.linear.z = 0
            manual_ctrl_pub.publish(vel)
        # X commands a negative pitch
        elif (key == 'x'):
            vel.linear.x = -SPEED
            vel.linear.y = 0
            vel.linear.z = 0
            manual_ctrl_pub.publish(vel)
        # E commands a positive roll
        elif (key == 'e'):
            vel.linear.x = 0
            vel.linear.y = SPEED
            vel.linear.z = 0
            manual_ctrl_pub.publish(vel)
        # D commands a negative roll
        elif (key == 'd'):
            vel.linear.x = 0
            vel.linear.y = -SPEED
            vel.linear.z = 0
            manual_ctrl_pub.publish(vel)
        # If the key is one of the exit keys, close the program
        elif (key in exitKeys):
            break
        # If no command is pressed, then make sure not to affect the drone
        elif (vel.linear.z != 0 or vel.linear.x != 0 or vel.linear.y != 0):
            vel.linear.z = 0
            vel.linear.x = 0
            vel.linear.y = 0
            manual_ctrl_pub.publish(vel)
