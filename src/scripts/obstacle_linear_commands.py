#!/usr/bin/env python

# 2018 11 13 LiuYC SOLab
# Add the command to control moving obstacle

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, termios, tty
import numpy as np
import time


flag_move = 0

obsParam = {
        'f':(0.5), # diff increase 0.5 meter
        'd':(-0.5), # diff decrease 0.5 meter
        's':(0.2), # speed up * 1.2 
        'a':(0.2), # slow down * 1.2
           }

pub_rate = 100.0 # publish rate in Hz
speed = 50.0 # normal speed
diff = 2.0 # from -1 to 1, the moving range of obstacle

obs_vel = [3.0, -3.0]
# current direction of obstacle, obs0, obs1 respectivelly.
# default obs0 --> (1); obs1 <-- (-1)
obs_dir = [1, -1] 
obs_xpose = [0.0, 0.0]
go = True

# this is return the keyboard input
def getKey():

    tty.setraw(sys.stdin.fileno())
    # the time-out argument ([3]) will affect the pub_rate since it stands for how long it will wait for the three list to have some value.([rlist] [wlist] [xlist])
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    # The fellowing if keep system from waiting for input
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# update x coordinate of obs0
def get_obs0_odom(msg):
	global obs_xpose
	obs_xpose[0] = msg.pose.pose.position.x
 	#rospy.loginfo(obs_xpose)	
	
# check if endpoint is reached
def endpoint_check(xpose):
	global obs_dir
	endpoint = [0.0, 12.0]
	
	for i in range(len(xpose)):
	    if obs_dir[i] > 0:
		if xpose[i] > endpoint[1]:
		    obs_dir[i] = -obs_dir[i]
		    return True
		else: 
		    return False

	    elif obs_dir[i] < 0:
		if xpose[i] < endpoint[0]:
		    obs_dir[i] = -obs_dir[i]
		    return True
		else: 
		    return False

# Set obastacles' velocity
def set_obs_vel():
   
    global diff, speed, obs_dir, go

    pub_obs0_vel = rospy.Publisher('/obs0/cmd_vel', Twist, queue_size=5)
    sub_obs0_odom = rospy.Subscriber('/obs0/odom', Odometry, get_obs0_odom)
    rospy.init_node('obstacle_commands', anonymous=True)	
    rate = rospy.Rate(pub_rate) # default is 100
    
    while not rospy.is_shutdown():

	try:
	    key = getKey()
	    if key == 'f':
	        diff = diff + 0.5
	    elif key == 'd':
		diff == diff - 0.5
	    elif key == 's':
		speed == 1.2 * speed
	    elif key == 'a':
		speed == speed * 0.8
	    # ctrl+c will return '\x03'
	    elif key == 'c': # continue
		go = True
	    elif key == 'p': # pause
		go = False
	    elif key == '\x03':
		break

	finally:
 	    for i in range(len(obs_vel)):
	        obs_vel[i] = abs(obs_vel[i])*obs_dir[i]	
	        twist0 = Twist()
                twist0.linear.x = obs_vel[0]; twist0.linear.y = 0; twist0.linear.z = 0
                twist0.angular.x = 0; twist0.angular.y = 0; twist0.angular.z = 0

	    if endpoint_check(obs_xpose):
		go = False

	    if go:  
                pub_obs0_vel.publish(twist0)

	    else:
		pass

            rate.sleep()
#	    rospy.loginfo("count: %d",count)

if __name__ == '__main__':
    # part of getKeyUnix input
    settings = termios.tcgetattr(sys.stdin)
    try:
        set_obs_vel()
    except rospy.ROSInterruptException:
        pass
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

