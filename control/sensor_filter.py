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



class ProximityFilter(object):
    ''' Initializes a proximity filter with
        state model variance var_state,
        sensor model variance var_sensor
    '''
    def __init__(self, var_state, var_sensor):
        self.r = None
        
        # x is [r, dr/dt]
        self.kalman = KalmanFilter(dim_x=2, dim_z=1)
        
        # State transition matrix (assume state velocity is units per timestep
        self.kalman.F = np.array([[1., 1.],
                                  [0., 1.]])
        self.kalman.Q = Q_discrete_white_noise(dim=2, dt=1, var=var_state)
        self.kalman.R = var_sensor
        self.kalman.H = np.array([[1., 0.]])
        
        # This guess should go away immediately
        self.kalman.x = np.array([[0.], [0.]])
        self.kalman.P *= 500.
    
    def __call__(self, z, R=None):
        self.kalman.predict()
        self.kalman.update(z, R)
        return self.kalman.x[0]

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
    f = ProximityFilter(var_state, var_sensor)
    
    raw_pub = rospy.Publisher("raw_distance"+topic, Float64, queue_size=10)
    filtered_pub = rospy.Publisher("filtered_distance"+topic, Float64, queue_size=10)
    
    def callback(data):
        dist = sensor(data.data)
        
        if data.data:
            filtered = f(dist)
            raw_pub.publish(Float64(dist))
        else:
            # We don't trust the 0 values, but it's still information
            filtered = f(dist, var_sensor*20)
        
        filtered_pub.publish(Float64(filtered))
    
    sub = rospy.Subscriber(topic, Int8, callback)

def make_filters(topics):
    return [make_filter(topic) for topic in topics]

if __name__ == '__main__':
    rospy.init_node('sensor_filter')
    make_filters(sys.argv[1:])
    rospy.spin()
