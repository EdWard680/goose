#!/usr/bin/env python

from nav_msgs.msg import Odometry
from duckietown_msgs.msg import Twist2DStamped
import rospy
import os
import math

pub = None

v = 0.1
K = 0.3

def min_angle_diff(theta1, theta2):
    ret = theta1 - theta2
    return min(ret, ret - 2*math.pi, ret + 2*math.pi)

def callback(data):
    global pub
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    
    theta = math.atan2(z, w)*2
    r = math.sqrt((x**2)+(y**2))
    
    goal = math.atan2(-x, y)
    omega = v/r + K*min_angle_diff(theta+math.pi, goal)

    msg = Twist2DStamped()
    msg.v = v
    msg.omega = omega

    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("controller")
    pub = rospy.Publisher("/twist", Twist2DStamped, queue_size=10)
    this_duck = os.environ.get("ID")
    topic = "odometry/duck{}".format(this_duck)
    sub = rospy.Subscriber(topic, Odometry, callback)
    
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    
    msg = Twist2dStamped()
    msg.v = 0
    msg.omega = 0
    pub.publish(msg)
