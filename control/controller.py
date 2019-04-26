#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from duckietown_msgs.msg import Twist2DStamped
import rospy
import os
import math

def callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    
    theta = math.atan2(z/w)*2
    r = math.sqrt((x**2)+(y**2))
    v = 0.1
    omega = v/r

    msg = Twist2DStamped()
    msg.v = v
    msg.omega = omega

    pub = rospy.Publisher("/twist", Twist2DStamped, queue_size=10)
    pub.publish(msg)

if __name__ == '__main__':
    this_duck = os.environ.get("ID")
    topic = "odometry/duck{}".format(this_duck)
    sub = rospy.Subscriber(topic, Odometry, callback)
    rospy.spin()
