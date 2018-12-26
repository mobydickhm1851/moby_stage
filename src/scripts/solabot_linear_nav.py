#!/usr/bin/env python

# 2018 12 14 LiuYC SOLab
# Simple linear navigation for solabot

import roslib
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
# import modult for n-d array
from std_msgs.msg import Float32MultiArray
from costmap_module.numpy_nd_msg import numpy_nd_msg
from costmap_module import update 
#import costmap.update_costmap 
import time




# global variables from "update.py" in costmap_module
car_vel = np.zeros((1, 2))
car_pose = update.car_pose
map_size = update.map_size
map_res = update.map_res
t_res = update.t_res



# ================================ Navigation ================================= #

# NOTE: not sure how to set the dimension of the matrix. For now, use the width of the intersection as the dimensions. (which is 80)

# DEF=> col_prob_matrix stands for: (1, 80)array in y-direction (col_prob_matrix[1]) from the current car_pose of every unit "future costmaps" (col_prob_matrix[0])

col_prob_matrix = np.zeros((80, 80))



def reset_prob_matrix():
    global col_prob_matrix 

    col_prob_matrix = np.zeros((80, 80))



def update_prob_matrix():
    
    # reset the matrix
    reset_prob_marix()

    pass




# Given a realworld coordinate return the collision probability
def get_col_prob(t, cor_lst):

    idx = update.pose_to_costcor(cor_lst)
    costmap = update.costmap
    col_prob = costmap[t][idx[0][1]][idx[0][0]]
    
    return col_prob


# calculate the differentiate of collision probability on a coordinate in time t_res
def col_prob_diff(t0, cor_lst):
    
    prob0 = get_col_prob(t0, cor_lst)
    prob1 = get_col_prob(t0 + t_res, cor_lst)

    prob_diff = prob1 - prob0

    return prob_diff



# the distance to cross the obstacle ahead (in meter)
def get_cross_dist(cor_list):

    cor_list_temp = cor_list.copy()
    dist2cross = 0
    notfindit = True

    while notfindit:
        
        if dist2cross < int(np.ceil(map_size/map_res)):

            dist2cross += 1
            # NOTE 1-D only!!!! ([0, map_res]), to +X direction
            cor_list_temp[0] = cor_list_temp[0] + [0, map_res]
            
            if get_col_prob(0, cor_list_temp) == 0:

                notfindit = False

                return dist2cross * map_res 
            
            else:
                pass

        else:
            # raise error
            return -1


# the time from now to actual impact (col_prob == 1)
def get_impact_time(cor_list):

    costmap = update.costmap
    cor_list_temp = cor_list.copy()
    time_impact = 0
    notfindit = True

    while notfindit:
        
        if time_impact + 1 < costmap.shape[0]:

            time_impact += 1

        # NOTE 1-D only!!!! horizontal search only, search direction should be along vel-vector

            if get_col_prob(time_impact, cor_list_temp) == 1:

                notfindit = False

                return time_impact * t_res 
            
            else:

                pass

        else:

            return 0



# ============================================================================ #

def main():
    
    global car_vel

    # car's initial movement 
    car_vel[0][1] = update.car_init_y_vel
    print("car vel begining of main() : {0}".format(car_vel))

# Initialize the node    
    rospy.init_node('solabot_commands', anonymous=True)	
    
# ROS Publishers
    # publish as numpy array using numpy_msg
    pub_costmap = rospy.Publisher('/costmap', numpy_nd_msg(Float32MultiArray), queue_size=5)
    # data of the "car" 
    pub_car_vel = rospy.Publisher('/car/cmd_vel', Twist, queue_size=5)
    
# ROS Subscribers
    rospy.Subscriber('/car/base_pose_ground_truth', Odometry, update.update_car_odom)
    # data of the "obstacles"
    obs_list = update.obs_list
    for i in range(len(obs_list)):
        rospy.Subscriber('/{0}/base_pose_ground_truth'.format(obs_list[i]), Odometry,update.update_obs_odom,(i))


    rate = rospy.Rate(20) # default is 100

    
    while not rospy.is_shutdown():

        try:
            pass

        finally:

            # update the costmap
            update.update_costmap()
            costmap = update.costmap
            pub_costmap.publish(data = costmap)
            



            # NOTE different drive_mode ?  
            prob_thresh = 0.0  # smaller the thresh, more careful the driver is	    
            # max acceleration (m/t_res)
            accele = 0.1  #(1m/s)
            # avoid by prediction
            lh_dist = - car_vel[0][1]**2 / (- accele / t_res)   # look ahead distance: able to stop with full break


            # get col_prob
            car_pose = update.car_pose
            lh_pose = car_pose[0] + [0, lh_dist]
            prob = get_col_prob(0, np.array([lh_pose]))

            print("now the col-prob {0}m ahead is:{1}.\n".format(lh_dist, prob))


            # NOTE: Different drive mode??? now is energy-saving mode
            # avoid obstacle while try to maintain the initial(target)speed
            if prob != 0: 

                cross_dist = get_cross_dist(np.array([lh_pose]))

                if update.get_t_ahead() > 0:

                    time_impact = get_impact_time(np.array([lh_pose]))
                
                    # whether car will collide with obs if it keeping at this car_vel
                    if cross_dist - car_vel[0][1] * time_impact > 0:

                        # NOTE: assume const. acceleration ! a = 1m/s (0.1m/t_res)
                        # calculate the root of t: 1/2*a*t**2 + V_0*t - S = 0
                        roots = np.roots([1/2 * (accele / t_res), car_vel[0][1], - cross_dist])
                        # only one positive root since S > 0
                        root_t = np.max(roots)

                        # accelerate OR decelerate
                        if root_t < time_impact:
                            # accelerate
                            car_vel[0][1] += accele

                        else :
                            # decelerate
                            car_vel[0][1] -= accele

                    else:
                        # keep moving can pass the obstacle
                        pass

            # NO OBSTACLE AHEAD
            else:
                
                if car_vel[0][1] + accele <= update.car_init_y_vel:

                    car_vel[0][1] += accele

                elif car_vel[0][1] - accele >= update.car_init_y_vel:
                    
                    car_vel[0][1] -= accele

            twist = Twist()
            twist.linear.x = 0; twist.linear.y = car_vel[0][1]; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            print("car vel begining before publish : {0}".format(car_vel))
            
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
