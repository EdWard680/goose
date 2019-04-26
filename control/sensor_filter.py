#!/usr/bin/env python3
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import sys
import rospy
from std_msgs.msg import Int8, Float64

# Converts a RSSI bluetooth readings to their derived distance from experimental data
class RssiConverter(object):
    # Signal propogation exponent
    n = 2
    
    # Initializes a converter with a reference measurement of A0 at d0
    def __init__(self, d0, A0):
        self.d0 = d0
        self.A0 = A0
    
    # Converts the passed in rssi value to the distance that emitted it
    def __call__(self, rssi):
        return self.d0 * 10**((rssi - self.A0) / (-10*RssiConverter.n))


def param_or_default(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return default

def make_filter(topic):
    A0 = param_or_default(topic+"/A0", 15.67)
    var_sensor = param_or_default(topic+"var_sensor", .139)
    var_state = param_or_default(topic+"var_state", .00000000001) # meters
    
    sensor = RssiConverter(.5, A0)
    
    raw_pub = rospy.Publisher("raw_distance"+topic, Float64, queue_size=10)
    
    def callback(data):
        dist = sensor(data.data)
        
		raw_pub.publish(Float64(dist))
    
    sub = rospy.Subscriber(topic, Int8, callback)

def make_filters(topics):
    return [make_filter(topic) for topic in topics]

if __name__ == '__main__':
    rospy.init_node('sensor_filter')
    make_filters(sys.argv[1:])
    rospy.spin()
