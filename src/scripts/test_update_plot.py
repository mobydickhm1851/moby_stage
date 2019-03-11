
#!/usr/bin/env python

# 2018 12 18 LiuYC SOLab
# Plotting costmap

import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import figaspect

# debug
import pdb





tryme = "testtesttest"

def costmap_plot():

# using costmap_plot() to plot the background first (with resolution of (map_size/map_res)^2), then in update_plot the color(zz, as probability) is updated.
    

    w, h = figaspect(.8)
    fig = plt.figure(figsize=(w,h))

    # comma here is to prevent the arg been passed unpacked, instead of as an arg
    ani = animation.FuncAnimation(fig, update_plot, interval = 10)
    plt.show()





def update_plot(i):
    global tryme


    ### text plot
    plt.clf()
    num = np.random.random(1)
    tryme = "{0}".format(num)
    plt.text(.5,.5, tryme, va='center')
    plt.title("car0's Costmap for Obsticles")
    plt.legend(scatterpoints = 1, loc='lower left')




if __name__ == '__main__':
   
    costmap_plot()
