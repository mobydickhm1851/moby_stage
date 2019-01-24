#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    while True:
        print("Now the key in test2 is:{0}".format(getKey()))
        if (getKey()=='\x03'):
            break
                

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
