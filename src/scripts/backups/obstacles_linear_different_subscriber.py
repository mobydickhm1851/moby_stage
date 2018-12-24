#!/usr/bin/env python

# 2018 11 13 LiuYC SOLab
# Add the command to control moving obstacle

import roslib
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
        's':(1.2), # speed up * 1.2
        'a':(0.8), # slow down * 0.8
           }
	
endpoints = [0.0, 12.0]
obs_vel = [1.0, -1.0]
# current direction of obstacle, obs0, obs1 respectivelly.
# default obs0 --> (1); obs1 <-- (-1)
obs_dir = [1, -1] 
obs_xpose = [0.0, 0.0]
go = True

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
def get_obs0_odom(msg):
	global obs_xpose
	obs_xpose[0] = msg.pose.pose.position.x
	
# update x coordinate of obs1
def get_obs1_odom(msg):
	global obs_xpose
	obs_xpose[1] = msg.pose.pose.position.x

# check if endpoint is reached
# IMPORTANT!! use tf transform or anyother way to get 'world coordinate', so we will know when either obstacle reach the boundaries. Now it's just a temp fix.
def endpoint_check(xpose):
	global obs_dir, endpoints
	
	if obs_dir[0] > 0 and xpose[0] > endpoints[1] or obs_dir[0] < 0 and xpose[0] < endpoints[0]:
	    obs_dir = [-i for i in obs_dir]
	    return True
        else: 
	    return False
'''
	    elif obs_dir[i] < 0:
		if abs(xpose[i]) < endpoints[0]:
		    obs_dir[i] = -obs_dir[i]
		    return True
		else: 
		    return False
'''
# Set obastacles' velocity
def set_obs_vel():
   
    global obs_dir, go, obs_vel, endpoints

    rospy.init_node('obstacle_commands', anonymous=True)	

    name0 = rospy.get_param('~obs0_name')
    name1 = rospy.get_param('~obs1_name')

    pub_obs0_vel = rospy.Publisher('/%s/cmd_vel' % name0, Twist, queue_size=5)
    pub_obs1_vel = rospy.Publisher('/%s/cmd_vel' % name1, Twist, queue_size=5)
    sub_obs0_odom = rospy.Subscriber('/%s/odom' % name0, Odometry, get_obs0_odom)
    sub_obs1_odom = rospy.Subscriber('/%s/odom'% name1, Odometry, get_obs1_odom)
    rate = rospy.Rate(20) # default is 100
    
    while not rospy.is_shutdown():

	try:
	    key = getKey()
#	    if key == 'f':
#	        endpoints[1] += obsParam['f']
#	    elif key == 'd':
#		endpoints[0] += obsParam['d']
	    if key == 's':
		obs_vel = [x*obsParam['s'] for x in obs_vel]
	    elif key == 'a':
		obs_vel = [x*obsParam['a'] for x in obs_vel]
		rospy.loginfo(obsParam['a'])
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

	    # twist0 for obs0
            twist0 = Twist()
       	    twist0.linear.x = obs_vel[0]; twist0.linear.y = 0; twist0.linear.z = 0
            twist0.angular.x = 0; twist0.angular.y = 0; twist0.angular.z = 0


	    # twist1 for obs1
            twist1 = Twist()
       	    twist1.linear.x = obs_vel[1]; twist1.linear.y = 0; twist1.linear.z = 0
            twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = 0

	    if endpoint_check(obs_xpose):
		go = False

	    if go:  
                pub_obs0_vel.publish(twist0)
                pub_obs1_vel.publish(twist1)

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

