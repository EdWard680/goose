#!/usr/bin/env python3
from pydoc import locate
import rospy
import std_msgs.msg
from rospy import AnyMsg
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
# ax2 = ax1.twinx()
line2, line1 = ax1.plot(xs, var_list, 'g-', xs, avg_list, 'b-')
# line2, = ax2.plot(xs, var_list, 'g-')

ax1.set_ylabel('RSSI (dB)')
# ax2.set_label('Variance')

def callback(data):
    global count, total, total_squared, xs, avg_list, line1, fig
    num = data.data
    if num != 0:
        count += 1
        total += num
        total_squared += num ** 2
        mean = total / count
        var = (total_squared - (total**2) / count) / count
        rospy.loginfo("num: {}, avg: {}, var: {}".format(num, mean, var))
        avg_list.append(mean)
        var_list.append(data.data)
        xs.append(count)
    # rospy.loginfo("data: {}".format(data.data))
        
        

def listener(topic, fig, line1):
    rospy.init_node('avg_and_var', anonymous=True)
    rospy.Subscriber(topic, locate(sys.argv[2]), callback)
    prev = count
    while not rospy.core.is_shutdown():
        if (count > prev and count <= 100):
            line1.set_xdata(xs)
            line1.set_ydata(avg_list)
            line1.set_label('Average')
            line2.set_xdata(xs)
            line2.set_ydata(var_list)
            line2.set_label('Measured')
            fig.canvas.draw()
            ax1.relim()
            ax1.autoscale_view()
            ax1.legend()
            # ax2.relim()
            # ax2.autoscale_view()
            fig.suptitle('Average RSSI Value and Measured RSSI Values', fontsize=16)
            ax1.set_xlabel('Samples')
            # ax1.set_ylabel('Average', color='b')
            # ax2.set_ylabel('RSSI (dB)', color='g')
            fig.canvas.flush_events()
            prev = count
        else:
            fig.canvas.flush_events()
        rospy.rostime.wallsleep(0.05)
    


if __name__ == '__main__':
    topic = str(sys.argv[1])
    print(locate(sys.argv[2]))
    listener(topic, fig, line1)
