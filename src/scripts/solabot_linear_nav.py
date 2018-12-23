#!/usr/bin/env python

# 2018 12 14 LiuYC SOLab
# Simple linear navigation for solabot

import roslib
import rospy
#from rospy_tutorials.msg import Floats
#from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#import sys, select, termios, tty
import numpy as np
# import modult for n-d array
from nd_msg.numpy_nd_msg import numpy_nd_msg
import time




# global variables
costmap = np.array(np.random.randn(2,250,250), dtype=np.float32)
# NOTE: make the costmap bigger incase the costmap is out of the bound (line 224)
# NOTE: the above should be considered as a problem
# initiate an array of (n,2) for obs_pose and obs_vel
obs_list=[]
obs_pose = []
obs_vel = []
map_size = 25   # default, should be replaced when param is imported
map_res = 0.1
t_res = 0.1   # prediction array for every 0.1s
car_init_vel = 0.6
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
#	rospy.loginfo("There is no moving object!")

    t_ahead = int(np.ceil(t_ahead / t_res))
 
    return t_ahead   # return int



def get_map_origin():
    map_x_num = map_size / map_res
    map_y_num = map_size / map_res
    origin = np.array([0, 0])
    origin[0] = int (map_x_num / 2)
    origin[1] = int (map_y_num / 2)
    
    return origin  # (2, ) array


def get_costmap_val( t, row_idx, col_idx):
    
    cost_val =  costmap[t][row_idx[:, None], col_idx]
    
    return cost_val   # probability of collision happening


# input should be a (N, ) array
def set_costmap_val( t, row_idx, col_idx, set_val):
    global costmap
    costmap[t][row_idx[:, None], col_idx] = np.ones((len(row_idx), len(col_idx))) * set_val
    

def change_costmap_val( t, row_idx, col_idx, set_val):
    
    val_old = get_costmap_val(t, row_idx, col_idx)

    # save memory (?)
    if not np.any(val_old):  # if all is zero
	set_costmap_val(t, row_idx, col_idx, set_val)
    # NOTE only obs, so 'or' is fine
    else:   # should between 0 and 1 (probability)
	# P(A or B) = P(A) + P(B) - P(A)P(B)
	set_val = val_old + set_val - val_old * set_val 
	set_costmap_val(t, row_idx, col_idx, set_val)



# NOTE: now it's 1-D, set square castmap around obstacle
def cost_function(dist_to_obs, obs_vel):
 
    obs_vel = abs(obs_vel)
  
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


# change coordination from map to costmap
# NOTE: function pose_to_costcore change list value !!!
# NOTE: INPUT has to be array!
def pose_to_costcor(cor_arr):
    costmap_origin = get_map_origin()
    cor_arr_ = np.array(cor_arr, dtype=int)

    # using np.ceil to avoid unmatching array when doing round up (line 230)
    cor_arr_[:,0] = np.array(np.ceil( costmap_origin[0] + cor_arr[:,0] / map_res), dtype=int)
    # y increase in  opposite direction
    cor_arr_[:,1] = np.array(np.ceil( costmap_origin[1] - cor_arr[:,1] / map_res), dtype=int)

    return cor_arr_  # (2, ) int array


# ======================================== Update Costmap ======================================== #

def get_obs_cost_pose(t):
    arr, arr_all = [], []

    # range covered by sizeof obs + car (so the car become a point)	
    cost_range_cor = np.around( car_dim + obs_dim, int(abs(np.log10(map_res))))   
    # from map cor to cost cor (index)
    cost_range_index = int(np.ceil(cost_range_cor / map_res))

    # update pose for every t ([0] of costmap)
    pose = obs_pose + obs_vel * t * t_res
    upper = pose_to_costcor(pose) + cost_range_index  # bot-right 
    #NOTE: function pose_to_costcore change list value !!!
    pose = obs_pose + obs_vel * t * t_res
    lower = pose_to_costcor(pose) - cost_range_index  # top-left

    for obs_num in range(len(upper)):
	# index: y_min to y_max
        row_idx = np.array([np.arange(lower[obs_num][1], upper[obs_num][1]+1)]) 
        # index: x_min to x_max
        col_idx = np.array([np.arange(lower[obs_num][0], upper[obs_num][0]+1)])
	arr = np.array([np.append(row_idx, col_idx, 0)])

        if obs_num == 0:
	    arr_all = arr
	
	else:
	    arr_all = np.append(arr_all, arr, 0)

    return arr_all


def reset_costmap(t_lh):
    global costmap 
    # default a square map
    map_x_num = map_size / map_res
    map_y_num = map_size / map_res
    
    # t_lh is not None and not < 0
    if t_lh > 0 :        
	# (t, x, y)array
    	costmap = np.zeros((t_lh + 1, map_x_num, map_y_num), dtype=np.float32)     
        return True

    # if no prediction is needed, generate local costmap
    else: 
	# (1, x, y)array, local costmap
    	costmap = np.zeros(( 1, map_x_num, map_y_num), dtype=np.float32)      
        return False
	

def update_costmap():
    arr_idx = np.array([])

    # check the costmap around when static
#    t_ah = get_t_ahead()
    t_ah = 1

    reset_costmap(t_ah)

    for t in range(t_ah + 1):
	     
	# Check if any obstacle will be out of range for some t in the prediction
	# NOTE: the -1 here is used to prevent out of borders
       	#pose = obs_pose + obs_vel * t * t_res
 	#if np.all(abs(pose) < (map_size / 2 - obs_dim - car_dim - 1)): 
	    
	
	    arr_idx = get_obs_cost_pose(t)
	 
            for obs_num in range(len(obs_pose)):
	    
	    	# Probability of collision is 1 for where the obstacle is (obs_dim + car+dim) 
	    	change_costmap_val( t, arr_idx[obs_num][0], arr_idx[obs_num][1], 1)

		# check if prediction is needed
		# NOTE: only x-direction is considered
		#if np.all([obs_vel[obs_num][0]]):
		if t_ah > 0:

 	    	# Create Collision Probability in front and rear of obstacles
	    	# NOTE: Consider x-vel only ([0]), should be on the direction of the vel-Vector
	    
    	            # where the cost function = 0
	    	    max_front = int(np.ceil(np.sqrt(abs(obs_vel[obs_num][0]) / front_factor) / map_res))
	    	    max_rear = int(np.ceil(np.sqrt(abs(obs_vel[obs_num][0]) / rear_factor) / map_res))

	  	    # The right and left index
	 	    r_idx = arr_idx[obs_num][1][-1]  # upper x
		    l_idx = arr_idx[obs_num][1][0]   # lower x
	    
	   	    # Velocity direction check, 1 if > 0 ; -1 if < 0 
	            v_check = abs(obs_vel[obs_num][0]) / obs_vel[obs_num][0]

	            # For cells on the RIGHT of the obstacle
	            # (vel > 0 : FRONT; vel < 0 : REAR)	
   	            for r_cell in range(r_idx, r_idx + max_front + 1):
		
		    	dist_to_obs = (r_cell - r_idx) * map_res * v_check
		    	cost_val = cost_function(dist_to_obs, obs_vel[obs_num][0])

		    	# check if the index is out of range
		    	if bordercheck(r_cell):
		            # change the costval col-wise
		            change_costmap_val( t, arr_idx[obs_num][0], [r_cell], cost_val)
		    	else :
		            break

	            # For cells on the LEFT of the obstacle
	            # (vel > 0 : REAR; vel < 0 : FRONT)	
		    # NOTE: range of max_front can cover max_rear, the other way is not working
	            for l_cell in range(l_idx - max_front, l_idx + 1 ):
		
	            	dist_to_obs = (l_cell - l_idx) * map_res * v_check
		    	cost_val = cost_function(dist_to_obs, obs_vel[obs_num][0])

		    	# check if the index is out of range
		    	if bordercheck(l_cell):
		            # change the costval col-wise
		            change_costmap_val( t, arr_idx[obs_num][0], [l_cell], cost_val)
		    	else :
		            pass
	    else:
		break


# ================================================================================================ #

# ================================ Navigation ================================= #
def get_col_prob(t, cor_lst):
    idx = pose_to_costcor(cor_lst)
    col_prob = costmap[t][idx[0][1]][idx[0][0]]
    return col_prob


# calculate the differentiate of collision probability
def col_prob_diff(cor, t1, t2):
    pass    


# ============================================================================ #

def main():
    global obs_pose, obs_vel, costmap, map_size, map_res

    obs_list = ['obs0', 'obs1']
    obs_pose = np.zeros((len(obs_list), 2))
    obs_vel = np.zeros((len(obs_list), 2))

    rospy.init_node('solabot_commands', anonymous=True)	
    car_vel[0][1] = rospy.get_param('~init_vel', car_init_vel) # default is 1.0
    map_res = rospy.get_param('~cmap_res', 0.1) # default is 1.0
    map_size = rospy.get_param('~cmap_size', 25) # default is 25
    # publish as numpy array using numpy_msg
    pub_costmap = rospy.Publisher('/costmap', numpy_nd_msg(Float32MultiArray), queue_size=5)
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

            update_costmap()
    	    pub_costmap.publish(data = costmap)
	    
	    car_vel[0][1] = 0
	    
	    lh_dist = 2   # look ahead distance

	    lh_pose = car_pose[0] + [0, lh_dist]
	    prob = get_col_prob(0, np.array([lh_pose]))

	    prob_thresh = 0.6  # smaller the thresh, more careful the driver is	    
	               
	    if prob != 0: 
	        rospy.loginfo("now the col-prob {0}m ahead is:{1}. pose and pose transformed are {2}, {3}".format(lh_dist, prob, lh_pose, pose_to_costcor(np.array([lh_pose]))))

	    '''
            if prob > prob_thresh:
	        car_vel = 0
	    else:
		pass

	    else:
		# keep moving
		pass	
	    '''

            twist = Twist()
       	    twist.linear.x = 0; twist.linear.y = car_vel[0][1]; twist.linear.z = 0
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
