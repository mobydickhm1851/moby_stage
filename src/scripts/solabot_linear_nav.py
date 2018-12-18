#!/usr/bin/env python

# 2018 12 14 LiuYC SOLab
# Simple linear navigation for solabot

import roslib
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys, select, termios, tty
import numpy as np
#import time

# global variables
costmap = np.array([], dtype=np.float32)
# NOTE: make the costmap bigger incase the costmap is out of the bound (line 224)
# NOTE: the above should be considered as a problem
map_size = 30   # default, should be replaced when param is imported
map_res = 0.1
t_res = 0.1   # prediction array for every 0.1s
car_vel = 0.5
car_init_vel = 0.6
obs_vel = []
obs_pose = []
car_vel = np.array([[0.0, 0.0]])
car_pose = np.array([[0.0, 0.0]])
car_dim = 1.1   # radius, think of as a circle, NOTE: should be modified
obs_dim = 0.67   # radius, think of as a circle, NOTE: should be modified
# factors for cost function
front_factor = 0.0625
rear_factor = 0.25




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
    # return if at least one obs is still moving 
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

    else:
	t_ahead = 0
	rospy.loginfo("There is no moving object!")

    t_ahead = int(np.ceil(t_ahead / t_res))
 
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


def get_costmap_val( t, row_idx, col_idx):
    
    cost_val =  costmap[t][row_idx[:, None], col_idx]
    
    return cost_val   # probability of collision happening


# input should be a (N, ) array
def set_costmap_val( t, row_idx, col_idx, set_val):
    global costmap
    
    costmap[t][row_idx[:, None], col_idx] = np.ones((len(row_idx), len(col_idx))) * set_val
    

def change_costmap_val( t, row_idx, col_idx, set_val):
    global costmap
    
    val_old = get_costmap_val(t, row_idx, col_idx)
    set_val = np.ones((len(row_idx), len(col_idx))) * set_val

    # save memory (?)
    if not np.any(val_old):  # if all is zero
	set_costmap_val(t, row_idx, col_idx, set_val)

    else:   # should between 0 and 1 (probability)
	# P(A or B) = P(A) + P(B) - P(A)P(B)
	set_val = val_old + set_val - val_old * set_val 
	set_costmap_val(t, row_idx, col_idx, set_val)
        

# NOTE: now it's 1-D, set square castmap around obstacle
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


# check if the index is within the border
def bordercheck(idx):
    max_idx = map_size / map_res

    if idx < max_idx:
	return True

    else:
	return False

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
    cost_range_index = int(np.ceil(cost_range_cor / map_res))
#    rospy.loginfo("cost_range:{0}".format(cost_range_index))
    costmap_origin = get_map_origin()

    #  update castmap in all t within t_ahead 
    t_ahead = get_t_ahead()

    if t_ahead > 0:
	pass
    else:
	t_ahead = 1  
    
    # NOTE -1 is a quick fix...	
    for t in range(t_ahead-1):
	# NOTE: this is linear motion prediction
        pose = obs_pose + obs_vel * t * t_res
     	#  for each obstacle
        for i in range(len(obs_pose)):
	    # check if the prediction pose of obstacle is out of range
	    # NOTE: the -1 here is used to prevent out of borders
	    if np.all(abs(pose[i]) < (map_size / 2 - obs_dim - car_dim - 1)): 
	        # from map cor to cost cor
	        upper = pose_to_costcor(pose[i] + cost_range_cor) 
	        lower = pose_to_costcor(pose[i] - cost_range_cor)  

	        row_idx = np.arange(upper[1], lower[1]+1)  # index: y_min to y_max
	        col_idx = np.arange(lower[0], upper[0]+1)  # index: x_min to x_max

		    # location that obstacle at has probability of collision equals to 1
  	        change_costmap_val( t, row_idx, col_idx, 1)
#		if t == 0 and i == 0:
#     	    	    rospy.loginfo("this is center and pose and rwo_idx col_idx: {0}{1}{2}{3}".format(get_map_origin(), pose[0],row_idx,col_idx))
#		rospy.loginfo("I did change the costmap{0}".format(t))

		# NOTE: only x-direction vel is considered ([i][0])
		if obs_vel[i][0] > 0:
		    # cost function: in front of the obstacle
		    # NOTE:5 meters ahead, this should cover the costfunction val that > 0
		    for j in range(upper[0] + 1, upper[0] + 50):
		        dist_to_obs = (j-upper[0]) * map_res
			cost_val = cost_function(dist_to_obs, obs_vel[i][0])

			# check if the index is out of range
			if bordercheck(j):
			    # change the costval row-wise
			    change_costmap_val( t, row_idx, [j], cost_val)
#			    rospy.loginfo("mom, i'm here!!!! obs_pose : {0}".format(obs_pose))
#			    rospy.loginfo("upper[0] and j : {0} {1}".format(upper[0],j))
			else :
			    pass

		    # cost function: rear of the obstacle
		    for k in range(lower[0] - 50, lower[0] - 1):
		        dist_to_obs = - (lower[0]-k) * map_res
			cost_val = cost_function(dist_to_obs, obs_vel[i][0])
			# check if the index is out of range
			if bordercheck(k):
			    # change the costval row-wise
			    change_costmap_val( t, row_idx, [k], cost_val)
#			    rospy.loginfo("dad, i'm here!!!!!!!!")
			else:
			    pass

		elif obs_vel[i][0] < 0:
		    for j in range(lower[0] - 50, lower[0] - 1):
		        dist_to_obs = (lower[0]-j) * map_res
			cost_val = cost_function(dist_to_obs, obs_vel[i][0])

			# check if the index is out of range
			if bordercheck(j):
			    # change the costval row-wise
			    change_costmap_val( t, row_idx, [j], cost_val)
#			    rospy.loginfo("mom, i'm here!!!! obs_pose : {0}".format(obs_pose))
			else :
			    pass

		    # cost function: rear of the obstacle
		    for k in range(upper[0] + 1, upper[0] + 50):
		        dist_to_obs = -(k-upper[0]) * map_res
			cost_val = cost_function(dist_to_obs, obs_vel[i][0])
			# check if the index is out of range
			if bordercheck(k):
			    # change the costval row-wise
			    change_costmap_val( t, row_idx, [k], cost_val)
#			    rospy.loginfo("dad, i'm here!!!!!!!!")
			else:
			    pass

	    else:
		pass

def get_col_prob(t, cor_lst):
    idx = pose_to_costcor(cor_lst)
    col_prob = costmap[t][idx[0]][idx[1]]

    return col_prob


# calculate the differentiate of collision probability
def col_prob_diff(cor, t1, t2):
    pass    




'''
import matplotlib.pyplot as plt

plt.ion()
plt.figure()
plt.rcParams["figure.figsize"] = [100,100]

# input is a (x_num, y_num) array
def plot_array(arr):

    x = np.arange( arr.shape[0])
    y = np.arange( arr.shape[1])

    for i in range(len(x)):
        x_plt = np.ones((1,len(x))) * i
        y_plt = y
        # values of arr should be from 0 to 1
        z = np.transpose(arr)[i]
        # grey scale of 'binary', 1 is black, 0 is white
        plt.scatter(x_plt, y_plt, c = z, s = 1, alpha = 0.5, cmap='binary') 
'''	    


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
    # publish as numpy array using numpy_msg
    pub_costmap = rospy.Publisher('/costmap', numpy_msg(Floats), queue_size=200)
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

	    # slow down or accelerate
            update_costmap()
	    prob = get_col_prob(0, car_pose[0]+2)

	    if prob != 0.0:
	        car_vel = 0

	    else:
		car_vel = car_init_vel
	    rospy.loginfo("collision probability 5m ahead :{0} ".format(prob))





# publicated data is not right
#		pub_costmap.publish(costmap[0])

# visualization, not finished
#    		plot_array(costmap[0])
#    		plt.draw()
#    		plt.pause(0.01)
#		plt.clf()

#	    else:
		# keep moving
#		pass	


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
