#!/usr/bin/env python

from __future__ import print_function

# Load ROS
import roslib; roslib.load_manifest('gtddp_drone')
import rospy

# ROS Messages
from std_msgs.msg import Empty

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


# Main Function
if __name__=="__main__":
    # Enable Termios
    settings = termios.tcgetattr(sys.stdin)

    # Set up ROS publishers
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=10)   # emergency land the drone

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
        if(key == ' '):
            land_msg = Empty()
            land_pub.publish(land_msg)
        # If the key is one of the exit keys, close the program
        elif(key in exitKeys):
            break
    