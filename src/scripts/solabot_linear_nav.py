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
# NOTE: make the costmap bigger incase the costmap is out of the bound (line 224)
# NOTE: the above should be considered as a problem
map_size = 30   # default, should be replaced when param is imported
map_res = 0.1
car_vel = 1.0
car_init_vel = 1.0
obs_vel = []
obs_pose = []
car_vel = np.array([[0.0, 0.0]])
car_pose = np.array([[0.0, 0.0]])
car_dim = 1.1   # radius, think of as a circle, NOTE: should be modified
obs_dim = 0.67   # radius, think of as a circle, NOTE: should be modified
# factors for cost function
front_factor = 1.0
rear_factor = 4.0




# update obs_vel and obs_pose
# NOTE: Now it's 1-D, only on x-direction
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
# NOTE: Now it's 1-D, only on y-direction
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
    

def get_dists():
    dists = []
    # Euclidean distance    
    dists =np.sqrt( np.sum( np.power( np.subtract( obs_pose, car_pose), 2), 1))
    
    return dists     # (1,N) array


# t_ahead is the maximum useful predict time
# NOTE: now it's only 1-D, so this work. It can't cover 2-D situation though.
# NOTE: car_vel is not considered
def get_t_ahead():
    # obs_vel is a (N,2) array (vector), find the absolute of each vector first
    dists = get_dists()
    obs_val = np.sqrt( np.sum( np.power( obs_vel, 2), 1))
    t_ahead = 0.0
    
    if np.count_nonzero(obs_vel):

	for i in range(len(obs_val)):
	    if obs_val[i] == 0:
		obs_val[i] = -1
	    else:
		pass
    	
	t_ahead = np.amax( dists / obs_val)
	t_ahead = int(np.ceil(t_ahead))

    else:
	t_ahead = int(0)
	rospy.loginfo("There is no moving object!")

    return t_ahead   # return int



def reset_costmap():
    global costmap 
    # default a square map
    map_x_num = map_size / map_res
    map_y_num = map_size / map_res
    t_lh = get_t_ahead()	    
    
    # t_lh is not None and not < 0
    if t_lh > 0 :        
	t_lh = np.ceil(t_lh)
    	costmap = np.zeros((t_lh, map_x_num, map_y_num))     # (t, x, y)array
        return True

    # if no prediction is needed, generate local costmap
    else : 
    	costmap = np.zeros((1, map_x_num, map_y_num))     # (1, x, y)array, local costmap
        return False
	

def get_map_origin():
    map_x_num = map_size / map_res
    map_y_num = map_size / map_res
    origin = np.array([0, 0])
    origin[0] = int (map_x_num / 2)
    origin[1] = int (map_y_num / 2)
    
    return origin  # (2, ) array


# change coordination from map to costmap
def pose_to_costcor(cor_lst):
    costmap_origin = get_map_origin()
    cost_cor = np.array([0, 0])
    # NOTE: using np.ceil to avoid unmatching array when doing round up (line 230)
    cost_cor[0] = int(np.ceil( costmap_origin[0] + cor_lst[0] / map_res))
    # NOTE: y increase in  opposite direction
    cost_cor[1] = int(np.ceil( costmap_origin[1] - cor_lst[1] / map_res))  

    return cost_cor  # (2, ) int array


def set_costmap_val( t, cor_lst, val):
    
    cor = pose_to_costcor(cor_lst)
    costmap[t][cor[0]][cor[1]] = val

    
def get_costmap_val( t, cor_lst):
    
    cor = pose_to_costcor(cor_lst)
    val = costmap[t][cor[0]][cor[1]]
    
    return val   # probability of collision happening



def change_costmap_val( t, cor_lst, val):
    global costmap
    
    val_old = get_costmap_val(t, cor_lst)

    if val_old == 0:
	set_costmap_val(t, cor_lst, val)

    else:   # should between 0 and 1 (probability)
	# P(A or B) = P(A) + P(B) - P(A)P(B)
	val = val_old + val - val_old * val 
	set_costmap_val(t, cor_lst, val)
        

# now it's 1-D, set square castmap around obstacle
def cost_function(dist_to_obs, obs_vel):
    global front_factor, rear_factor
   
    if dist_to_obs >= 0:   # in front of the obstacle
	val = -(front_factor / obs_vel) * dist_to_obs**2 + 1

    else:    # behind the obstacle
	val = -(rear_factor / obs_vel) * dist_to_obs**2 + 1
	
    if val < 0 :
	return 0

    else:    # val should be between 0 and 1
	return val


def update_costmap():
    global costmap

    if reset_costmap():
	pass
 
    else:    
	rospy.loginfo("No prediction is needed, turn to local costmap now!")

    # NOTE: not the function of costmap is 1-D (line), should be modified into 2-D (circle)
    # range covered by sizeof obs + car (so the car become a point)	
    cost_range_cor = np.around( car_dim + obs_dim, int(abs(np.log10(map_res))))   
    # from map cor to cost cor (index)
    cost_range_index = int(cost_range_cor / map_res)
    rospy.loginfo("cost_range:{0}".format(cost_range_index))
    costmap_origin = get_map_origin()

    #  update castmap in all t within t_ahead 
    t_ahead = get_t_ahead()

    for t in range(t_ahead):
	# location that obstacle at has probability of collision equals to 1   
	# NOTE: this is linear motion prediction
        pose = obs_pose + obs_vel * t

     	#  for each obstacle
        for i in range(len(obs_pose)):
	    # check if the prediction is out of range
	    if np.all(abs(pose) < (map_size / 2 - obs_dim - car_dim)): 
	        # from map cor to cost cor
	        upper = pose_to_costcor(pose[i] + cost_range_cor) 
	        lower = pose_to_costcor(pose[i] - cost_range_cor)  

	        row_index = np.arange(upper[1], lower[1])  # index: y_min to y_max
	        col_index = np.arange(lower[0], upper[0])  # index: x_min to x_max

	        rospy.loginfo("pose, row_index, col_index:{0}{1}{2}".format(pose, row_index, col_index))

	        costmap[t][row_index[:, None], col_index] = np.ones((cost_range_index*2, cost_range_index*2))

	    else:
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
	    # time look ahead
	    t_lh = 0.0
	    t_lh = get_t_ahead()	    

	    cost_cor = pose_to_costcor([2,2])

	    rospy.loginfo("corrdination in costmap is :{0}".format(cost_cor))

	    update_costmap()

            twist = Twist()
       	    twist.linear.x = 0; twist.linear.y = car_vel; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	    if True:  
                pub_car_vel.publish(twist)

	    else:
		pass

            rate.sleep()


if __name__ == '__main__':

    try:
	main()
    except rospy.ROSInterruptException:
        pass
