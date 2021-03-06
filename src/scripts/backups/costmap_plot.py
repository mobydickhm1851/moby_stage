#!/usr/bin/env python

# 2018 12 18 LiuYC SOLab
# Plotting costmap

import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import Float32MultiArray
# import modult for n-d array
from nav_msgs.msg import Odometry
from costmap_module.numpy_nd_msg import numpy_nd_msg
from costmap_module import update
from matplotlib.figure import figaspect


costmap = np.array(np.random.randn(2,250,250))

def costmap_plot(arr):
   
    
    x_plt = np.zeros((1, arr.shape[1]))[0]
    y_plt = np.arange( arr.shape[1])

    for i in range(arr.shape[0]-1):
        x_plt = np.append(x_plt, np.ones((1,arr.shape[1]))[0]*i) 
        y_plt = np.append(y_plt, np.arange(arr.shape[1])[::-1])
    
    zz = arr.transpose().reshape(1,-1)[0]

    w, h = figaspect(1.)
    fig = plt.figure(figsize=(w,h))
    scat = plt.scatter(x_plt, y_plt, c=zz, marker="s",edgecolors="none", s = 10, alpha = 0.5, cmap='Greys') 

    car_pose_update = [[1, 1]]
    scat_car_pose = plt.scatter(car_pose_update[0][0], car_pose_update[0][1], c='red', marker='o', s = 100, alpha = 1)

    # comma here is to prevent the arg been passed unpacked, instead of as an arg
    ani = animation.FuncAnimation(fig, update_plot, fargs=(scat, scat_car_pose,))
    plt.show()

def update_plot(i, scat, scat_car_pose):
    arr = costmap[0]
    zz = arr.transpose().reshape(1,-1)[0]
    # set_array control the "color array"
    scat.set_array(zz)

    # get car_pose from subscriber
    car_pose_update = update.car_pose
    car_idx = update.pose_to_costcor(car_pose_update)
    # inverse y
    car_idx[0][1] = 125 - (car_idx[0][1] - 125)
    scat_car_pose.set_offsets([car_idx])
    #return scat,  

def callback(raw_arr):
    global costmap
    costmap = raw_arr.data


if __name__ == '__main__':
    
    rospy.init_node('costmap_plot', anonymous=True)	
    rospy.Subscriber('/costmap', numpy_nd_msg(Float32MultiArray), callback)
    rospy.Subscriber('/car/base_pose_ground_truth', Odometry, update.update_car_odom)

    costmap_plot(costmap[0])

    while not rospy.is_shutdown():

        try:
            pass 

        except rospy.ROSInterruptException:
            pass

