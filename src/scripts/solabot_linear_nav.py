#!/usr/bin/env python

# 2018 12 14 LiuYC SOLab
# Simple linear navigation for solabot

import roslib
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, termios, tty
import numpy as np
#import time

# global variables
costmap = np.array([])
map_size = 20   # default, should be replaced when param is imported
map_res = 0.1
car_vel = 1.0
car_init_vel = 1.0
obs_vel = []
obs_pose = []
car_vel = np.array([[0.0, 0.0]])
car_pose = np.array([[0.0, 0.0]])

# update obs_vel and obs_pose
# on phase one, only on x-direction
def update_obs_odom(msg, num):
    global obs_pose, obs_vel

    pose_x, pose_y = 0.0, 0.0
    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y
    obs_pose[num] = np.array([[pose_x, pose_y]]) 

    vel_x, vel_y = 0.0, 0.0
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    obs_vel[num] = np.array([[vel_x, vel_y]]) 



# update car_vel and car_pose
# on phase one, only on y-direction
def update_car_odom(msg):
    global car_pose, car_vel

    pose_x, pose_y = 0.0, 0.0
    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y
    car_pose[0] = np.array([[pose_x, pose_y]]) 
#    rospy.loginfo(car_pose)

    vel_x, vel_y = 0.0, 0.0
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    car_vel[0] = np.array([[vel_x, vel_y]]) 
    

def get_dists(poses, pose0):
    dists = []
    # Euclidean distance    
    dists =np.sqrt( np.sum( np.power( np.subtract( poses, pose0), 2), 1))
    
    return dists     # (1,N) array


def get_t_ahead(dists, obs_vel):
    # obs_vel is a (N,2) array (vector), find the absolute of each vector first
    obs_vel = np.sqrt( np.sum( np.power( obs_vel, 2), 1))
    
    if np.count_nonzero(obs_vel):
    	t_ahead_ = np.amin( dists / obs_vel)
	
	return t_ahead

    else:
	rospy.loginfo("There is no moving object!")



def reset_costmap():
    global costmap 
    # default a square map
    map_x_num = map_size / map_res
    map_y_num = map_size / map_res
    
    costmap = np.zeros((map_x_num, map_y_num))

def get_costmap(x, y):
    
    return prob_coli   # probability of collision happening
    
def update_costmap():
    pass

def main():
    global obs_pose, obs_vel

    obs_list = ['obs0', 'obs1']


    # initiate an array of (n,2) for obs_pose and obs_vel
    obs_pose = np.zeros((len(obs_list), 2))
    obs_vel = np.zeros((len(obs_list), 2))

    rospy.init_node('solabot_commands', anonymous=True)	

    car_vel = rospy.get_param('~init_vel', car_init_vel) # default is 1.0
    map_res = rospy.get_param('~cmap_res', 0.1) # default is 1.0
    map_size = rospy.get_param('~cmap_size', 20) # default is 1.0
    # data of the "car" 
    pub_car_vel = rospy.Publisher('/car/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber('/car/base_pose_ground_truth', Odometry, update_car_odom)
    # data of the "obstacles"
    for i in range(len(obs_list)):
	rospy.Subscriber('/{0}/base_pose_ground_truth'.format(obs_list[i]), Odometry, update_obs_odom,(i))

    rate = rospy.Rate(20) # default is 100
    while not rospy.is_shutdown():
	try:
	    pass
	finally:
	    lh_time = 0.0
	    lh_time = get_t_ahead( get_dists (obs_pose, car_pose), obs_vel)	    
	    rospy.loginfo(lh_time)

            twist = Twist()
       	    twist.linear.x = 0; twist.linear.y = car_vel; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	    if True:  
                pub_car_vel.publish(twist)
    		rospy.loginfo(obs_pose)

	    else:
		pass

            rate.sleep()


if __name__ == '__main__':

    try:
	main()
    except rospy.ROSInterruptException:
        pass
