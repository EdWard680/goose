#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import sys

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

count = 0
total = 0
total_squared = 0
xs = [0]
avg_list = [0]
var_list = [0]

plt.ion()
fig = plt.figure()
ax1 = fig.add_subplot(111)
ax2 = ax1.twinx()
line1, = ax1.plot(xs, avg_list, 'b-')
line2, = ax2.plot(xs, var_list, 'g-')

ax1.set_label('Average')
ax2.set_label('Variance')

def callback(data):
    global count, total, total_squared, xs, avg_list, line1, fig
    if data.data != 0:
        count += 1
        total += data.data
        total_squared += data.data ** 2
        mean = total / count
        var = (total_squared - (total**2) / count) / count
        rospy.loginfo("avg: {}, var: {}".format(mean, var))
        avg_list.append(mean)
        var_list.append(var)
        xs.append(count)
    # rospy.loginfo("data: {}".format(data.data))
        
        

def listener(topic, fig, line1):
    rospy.init_node('avg_and_var', anonymous=True)
    rospy.Subscriber(topic, Int8, callback)
    prev = count
    while not rospy.core.is_shutdown():
        if (count > prev):
            line1.set_xdata(xs)
            line1.set_ydata(avg_list)
            line2.set_xdata(xs)
            line2.set_ydata(var_list)
            fig.canvas.draw()
            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()
            ax1.set_ylabel('Average', color='b')
            ax2.set_ylabel('Variance', color='g')
            fig.canvas.flush_events()
            prev = count
        rospy.rostime.wallsleep(0.05)
    


if __name__ == '__main__':
    topic = str(sys.argv[1])
    listener(topic, fig, line1)
