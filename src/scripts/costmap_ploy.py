#!/usr/bin/env python

# 2018 12 18 LiuYC SOLab
# Plotting costmap

import matplotlib.pyplot as plt
import numpy as np
import time

plt.ion()
fig = plt.figure()


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
        plt.scatter(x_plt, y_plt, c = z, s = 100, alpha = 1, cmap='binary') 

while True:
    test_arr = np.ones((10,10))
    for j in range(len(test_arr[0])):
        test_arr[j] = test_arr[j]*j*np.random.rand()
    #test_arr = np.ones((10,10)) * np.random.rand()
    plot_array(test_arr)
    plt.draw()
    plt.pause(0.01)
    plt.clf()
    #plt.close()
    #time.sleep(1e-2)


print(test_arr)