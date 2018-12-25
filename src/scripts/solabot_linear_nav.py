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
car_vel = update.car_vel
car_pose = update.car_pose





# ================================ Navigation ================================= #
# Given a realworld coordinate return the collision probability
def get_col_prob(t, cor_lst):

    idx = update.pose_to_costcor(cor_lst)
    costmap = update.costmap
    col_prob = costmap[t][idx[0][1]][idx[0][0]]
    return col_prob


# calculate the differentiate of collision probability
def col_prob_diff(cor, t1, t2):
    pass    


# ============================================================================ #

def main():

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
            

            # car's initial movement 
            car_vel = update.car_vel
            car_vel[0][1] = update.car_init_y_vel


            # avoid by prediction
            lh_dist = 0   # look ahead distance
            # NOTE different drive_mode ?  
            prob_thresh = 0.98  # smaller the thresh, more careful the driver is	    
            # get col_prob
            car_pose = update.car_pose
            print("hahahaha: {0}".format(update.car_pose))
            lh_pose = car_pose[0] + [0, lh_dist]
            prob = get_col_prob(0, np.array([lh_pose]))

                    
            if prob != 0: 
#                print("now the col-prob {0}m ahead is:{1}. pose and pose transformed are {2}, {3}\n".format(lh_dist, prob, lh_pose, update.pose_to_costcor(np.array([lh_pose]))))

                if prob > prob_thresh:
                    car_vel[0][1] = 0
                else:
                    pass

            else:
            # keep moving
                pass	

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
