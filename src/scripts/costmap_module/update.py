#!/usr/bin/env python

# 2018 12 14 LiuYC SOLab
# costmap module for solabot_nav 

import roslib
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
# import modult for n-d array
from std_msgs.msg import Float32MultiArray
from .numpy_nd_msg import numpy_nd_msg
import time



# ====== global variable ======= #

# NOTE: make the costmap bigger incase the costmap is out of the bound (line 224)
# NOTE: the above should be considered as a problem
costmap = np.array(np.random.randn(2,250,250), dtype=np.float32)

# arrays of (n,2) for obs_pose and obs_vel
obs_list = ['obs0', 'obs1']
obs_pose = np.zeros((len(obs_list), 2))
obs_vel = np.zeros((len(obs_list), 2))
obs_dim = 0   # radius, think of as a circle, NOTE: should be modified

# map parameters from intersection.launch
map_res = 0
map_size = 0

# car parameters
car_vel = np.array([[0.0, 0.0]])
car_pose = np.array([[0.0, 0.0]])
car_dim = 0   # radius, think of as a circle, NOTE: should be modified

# time resolution
t_res = 0   # prediction array for every 0.1s

# factors for cost function
front_factor = 0
rear_factor = 0



# Initialize
map_res = rospy.get_param('~cmap_res', 0.1) # default is 1.0
map_size = rospy.get_param('~cmap_size', 25) # default is 25
car_init_y_vel = rospy.get_param('~init_vel', 1.0) # default is 1.0
car_dim = 1.1   
obs_dim = 0.67   
t_res = 0.1
front_factor = 0.0625
rear_factor = 0.25

# ============================== #




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
    obs_pose, car_pose

    dists = []
    # Euclidean distance    
    dists =np.sqrt( np.sum( np.power( np.subtract( obs_pose, car_pose), 2), 1))
    
    return dists     # (1,N) array



# t_ahead is the maximum useful predict time
# NOTE: now it's only 1-D, so this work. It can't cover 2-D situation though.
# NOTE: car_vel is not considered
def get_t_ahead():
    obs_vel, t_res

    # obs_vel is a (N,2) array (vector), find the absolute of each vector first
    # return if at least one obs is still moving 
    dists = get_dists()
    obs_vel_val = np.sqrt( np.sum( np.power( obs_vel, 2), 1))
    t_ahead = 0
    
    if np.count_nonzero(obs_vel):

        for i in range(len(obs_vel_val)):
            if obs_vel_val[i] == 0:
                obs_vel_val[i] = -1
            else:
                pass
            
            t_ahead = np.amax( dists / obs_vel_val)

    else:
        t_ahead = 0
       	# rospy.loginfo("There is no moving object!")

    t_ahead = int(np.ceil(t_ahead / t_res))
    
    return t_ahead   # return int



def get_map_origin():
    map_size, map_res

    map_x_num = map_size / map_res
    map_y_num = map_size / map_res
    origin = np.array([0, 0])
    origin[0] = int (map_x_num / 2)
    origin[1] = int (map_y_num / 2)
    
    return origin  # (2, ) array



def get_costmap_val( t, row_idx, col_idx):
    costmap

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
    front_factor, rear_factor, obs_vel
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
    map_size, map_res

    max_idx = map_size / map_res

    if idx < max_idx:
        return True

    else:
        return False



# change coordination from map to costmap
# NOTE: function pose_to_costcore change list value !!!
# NOTE: INPUT has to be array!
def pose_to_costcor(cor_arr):
    map_res
    costmap_origin = get_map_origin()
    cor_arr_ = np.array(cor_arr, dtype=int)

    # using np.ceil to avoid unmatching array when doing round up (line 230)
    cor_arr_[:,0] = np.array(np.ceil( costmap_origin[0] + cor_arr[:,0] / map_res), dtype=int)
    # y increase in  opposite direction
    cor_arr_[:,1] = np.array(np.ceil( costmap_origin[1] - cor_arr[:,1] / map_res), dtype=int)

    return cor_arr_  # (2, ) int array


# ======================================== Update Costmap ======================================== #

def get_obs_cost_pose(t):
    car_dim, obs_dim, map_res, t_res, obs_pose, obs_vel
    arr, arr_all = [], []

    # range covered by sizeof obs + car (so the car become a point)	
    cost_range_cor = np.around( car_dim + obs_dim, int(abs(np.log10(map_res))))   
    # from map cor to cost cor (index)
    cost_range_index = int(np.ceil(cost_range_cor / map_res))

    # update pose for every t ([0] of costmap)
    pose = obs_pose + obs_vel * t * t_res
    #NOTE: function pose_to_costcore change list value !!!
    pose_cost_cor = pose_to_costcor(pose)

    upper = pose_cost_cor + cost_range_index  # bot-right 
    lower = pose_cost_cor - cost_range_index  # top-left

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
    map_size, map_res
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
    obs_pose, obs_vel, t_res, map_size, map_res, obs_dim, car_dim, front_factor, rear_factor

    arr_idx = np.array([])
    # check the costmap around when static
    t_lh = get_t_ahead()
  
    #NOTE: restrict time dimension of costmap in 5 second
    if t_lh > 50:
        t_lh = 50


    reset_costmap(t_lh)

    for obs_num in range(len(obs_pose)):

        for t in range(t_lh + 1):
    
            arr_idx = get_obs_cost_pose(t)
        
            # Probability of collision is 1 for where the obstacle is (obs_dim + car+dim) 
            change_costmap_val( t, arr_idx[obs_num][0], arr_idx[obs_num][1], 1)

            # Check if any obstacle will be out of range for some t in the prediction
            # NOTE: the -1 here is used to prevent out of borders
            # check if prediction is needed
            # NOTE: only x-direction is considered
            #if np.all([obs_vel[obs_num][0]]):
            pose = obs_pose + obs_vel * t * t_res

            if np.all(abs(pose[obs_num]) < (map_size / 2 - obs_dim - car_dim - 1)) and t_lh > 0 and obs_vel[obs_num][0] != 0: 
        

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

