#!/usr/bin/env python

# 2018 12 13 LiuYC SOLab
# Add the command to control moving obstacle

import roslib
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, termios, tty
#import numpy as np
#import time


flag_move = 0

endpoints = [-6.0, 6.0]
obs_vel = 1.0   # default
# current direction of obstacle, obs0, obs1 respectivelly.
# default obs0 --> (1); obs1 <-- (-1)
obs_dir = 1   
obs_xpose = 0.0
go = True
        

obsParam = {
        #'f':(0.5), # diff increase 0.5 meter
        #'d':(-0.5), # diff decrease 0.5 meter
        's':(1.2), # speed up * 1.2
        'a':(0.8), # slow down * 0.8
           }
	

# this is return the keyboard input
def getKey():

    tty.setraw(sys.stdin.fileno())
    # the time-out argument ([3]) will affect the pub_rate since it stands for how long it will wait for the three list to have some value.([rlist] [wlist] [xlist])
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    # The fellowing if keep system from waiting for input
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# update x coordinate of obs0
def get_obs_pose(msg):
    global obs_xpose
    
    obs_xpose = msg.pose.pose.position.x
	

# check if endpoint is reached
# IMPORTANT!! use tf transform or anyother way to get 'world coordinate', so we will know when either obstacle reach the boundaries. Now it's just a temp fix.
def endpoint_check(xpose):
    endpoints    
    global obs_dir
	
    if obs_dir > 0 and xpose > endpoints[1] or obs_dir < 0 and xpose < endpoints[0]:
        obs_dir = - obs_dir
        return True
    else: 
        return False

# Set obastacles' velocity
def set_obs_vel():
    endpoints   
    global obs_dir, go, obs_vel

    #NOTE: this allows different node to have the same name, otherwise the latter will be kicked off!!
    rospy.init_node('obstacle_commands', anonymous=True)	

    name0 = rospy.get_param('~obs_name')
    pose0 = rospy.get_param('~init_pose')
    obs_vel = rospy.get_param('~init_vel', obs_vel) # default is 1.0

    sub_obs0_odom = rospy.Subscriber('/%s/base_pose_ground_truth' % name0, Odometry, get_obs_pose)

    pub_obs0_vel = rospy.Publisher('/%s/cmd_vel' % name0, Twist, queue_size=5)

    rate = rospy.Rate(20) # default is 100
    


    if pose0 > 0:
        obs_dir = -1
    elif pose0 < 0:
        obs_dir = 1
    #    rospy.loginfo("OUT OF THE LOOP ")

    while not rospy.is_shutdown():
	try:

	    key = getKey()
#	    if key == 'f':
#	        endpoints[1] += obsParam['f']
#	    elif key == 'd':
#		endpoints[0] += obsParam['d']
	    if key == 's':
		obs_vel = obsParam['s'] * obs_vel
	    elif key == 'a':
		obs_vel = obsParam['a'] * obs_vel
	    # ctrl+c will return '\x03'
	    elif key == 'c': # continue
		go = True
	    elif key == 'p': # pause
		go = False
	    elif key == '\x03':
		break

	finally:

	    if endpoint_check(obs_xpose):
		go = False

            obs_vel = abs(obs_vel)*obs_dir 
	    # twist0 for obs0
            twist0 = Twist()
       	    twist0.linear.x = obs_vel; twist0.linear.y = 0; twist0.linear.z = 0
            twist0.angular.x = 0; twist0.angular.y = 0; twist0.angular.z = 0


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

