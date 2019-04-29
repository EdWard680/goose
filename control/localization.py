#!/usr/bin/env python
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import numpy.linalg as la
import sys
import os
import rospy
import math
from std_msgs.msg import Int8, Float64
from nav_msgs.msg import Odometry
from duckietown_msgs.msg import Twist2DStamped

THIS_DUCK=int(os.environ.get("ID"))

# Converts a RSSI bluetooth readings to their derived distance from experimental data
class SensorModel(object):
    # Signal propogation exponent
    n = 2
    
    # Initializes a converter with a reference measurement of A0 at d0
    # And a variance in the sensor
    def __init__(self, d0, A0, variance):
        self.d0 = d0
        self.A0 = A0
        self.variance = variance
    
    def get_R(self):
        return np.array([[self.variance]])
    
    # Guesses an RSSI value given the system state
    # EKF Measurement Function 'h(x)'
    def __call__(self, our_pose, their_pose):
        delta = our_pose - their_pose
        distance = la.norm(delta[:2])
        
        return self.A0 + -10*SensorModel.n*math.log10(distance/self.d0)
    
    def get_state_jacobian(self, our_pose, their_pose):
        delta = our_pose - their_pose
        delta[2] = 0.
        
        return -10/math.log(10) / (la.norm(delta[:2])**2) * delta.T
    
    def get_noise_jacobian(self, our_pose, their_pose):
        # Assuming additive noise for now
        return np.array([[1]])

class StateModel(object):
    def __init__(self, vel_variance, turn_variance):
        self.vel_variance = vel_variance
        self.turn_variance = turn_variance
    
    def get_Q(self):
        return np.array([[self.vel_variance, 0], [0, self.turn_variance]])
    
    # Returns new state given old state (pose) and input (twist)
    def __call__(self, pose, twist, dt):
        new_pose = np.array([[0.], [0.], [0.]])
        
        if twist.omega:
            # Turning radius
            r = twist.v / twist.omega
            
            dtheta = twist.omega*dt
            
            new_pose[2] = pose[2] + dtheta
            new_pose[0] = pose[0] + r*(math.sin(new_pose[2]) - math.sin(pose[2]))
            new_pose[1] = pose[1] + r*(-math.cos(new_pose[2]) + math.cos(pose[2]))
        else:
            new_pose[2] = pose[2]
            new_pose[0] = pose[0] + twist.v*math.cos(pose[2])
            new_pose[1] = pose[1] + twist.v*math.sin(pose[2])
        
        return new_pose
    
    def get_state_jacobian(self, pose, twist, dt):
        new_pose = self(pose, twist, dt)
        if twist.omega:
            r = twist.v / twist.omega
            return np.array([[1, 0, r*(math.cos(new_pose[2]) - math.cos(pose[2]))],
                            [0, 1, r*(math.sin(new_pose[2]) - math.sin(pose[2]))],
                            [0, 0, 1]])
        else:
            return np.array([[1, 0, -twist.v*math.sin(pose[2])],
                             [0, 1, twist.v*math.cos(pose[2])],
                             [0, 0, 1]])
    
    def get_noise_jacobian(self, pose, twist, dt):
        if twist.omega:
            r = twist.v / twist.omega
            new_pose = self(pose, twist, dt)
            stheta = math.sin(pose[2])
            ctheta = math.cos(pose[2])
            snewtheta = math.sin(new_pose[2])
            cnewtheta = math.cos(new_pose[2])
            
            # Additive Controller White Noise
            '''
            return np.array([
                [(1/twist.omega)*(snewtheta - stheta), r*(dt*cnewtheta - snewtheta + stheta / twist.omega)],
                [(1/twist.omega)*(-cnewtheta + ctheta), r*(dt*snewtheta + cnewtheta - ctheta / twist.omega)],
                [0, 1]])
            '''
            
            # Multiplicative exponential Controller Noise ((e^w)*v)
            return np.array([
                [r*(snewtheta - stheta), -r*(snewtheta - stheta)],
                [r*(-cnewtheta + ctheta), -r*(-cnewtheta + ctheta)],
                [0, 1]])
        else:
            # Multiplicative exponential Controller Noise ((e^w)*v)
            return np.array([[twist.v*math.cos(pose[2]), -twist.v*math.cos(pose[2])],
                             [twist.v*math.sin(pose[2]), -twist.v*math.sin(pose[2])],
                             [0., 1.]])
        


class Filter(ExtendedKalmanFilter):
    def __init__(self, state0, state_mod, sensor_mod):
        ExtendedKalmanFilter.__init__(self, dim_x=3, dim_z=1, dim_u=2)
        
        self.x = state0
        self.P = np.zeros((3, 3))
        self.state_mod = state_mod
        self.sensor_mod = sensor_mod
        self.Q = self.state_mod.get_Q()
        self.R = self.sensor_mod.get_R()
    
    def predict(self, twist, dt):
        pose = self.x
        new_pose = self.state_mod(pose, twist, dt)
        self.x = new_pose
        
        Fx = self.state_mod.get_state_jacobian(pose, twist, dt)
        Fv = self.state_mod.get_noise_jacobian(pose, twist, dt)
        
        self.P = np.matmul(np.matmul(Fx, self.P), Fx.T) + np.matmul(np.matmul(Fv, self.Q), Fv.T)
        
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)
    
    def update(self, z, their_pose):
        ExtendedKalmanFilter.update(
            self,
            z,
            lambda x: self.sensor_mod.get_state_jacobian(x, their_pose),
            lambda x: self.sensor_mod(x, their_pose)
        )
        
def param_or_default(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return default

def make_filter():
    var_sensor = param_or_default("var_sensor", 1.5)
    var_vel = param_or_default("var_vel", .00000000001)
    var_steer = param_or_default("var_steer", 1)
    A0 = param_or_default("/A0", -40)
    x0 = param_or_default("/x0", 0)
    y0 = param_or_default("/y0", 0)
    theta0 = param_or_default("theta0", 0)
    
    state_mod = StateModel(var_vel, var_steer)
    sensor_mod = SensorModel(.5, A0, var_sensor)
    
    return Filter(np.array([[x0], [y0], [theta0]]), state_mod, sensor_mod)
    
class Duck(object):
    def __init__(self, duck_id, on_rssi):
        self.duck_id = duck_id
        self.odom_sub = rospy.Subscriber(
            "odometry/duck{}".format(duck_id),
            Odometry,
            self.on_odom)
        self.pose = np.array([[0.], [0.], [0.]])
        
        self.rssi_sub = rospy.Subscriber(
            "rssi/duck{}_duck{}".format(min(THIS_DUCK, duck_id), max(THIS_DUCK, duck_id)),
            Int8,
            lambda rssi: on_rssi(rssi.data, self.get_pose()))
    
    def on_odom(self, odom):
        self.pose = np.array([[odom.pose.pose.position.x], [odom.pose.pose.position.y], [0]])
    
    def get_pose(self):
        return self.pose

class Me(object):
    def __init__(self, ekf):
        self.ekf = ekf
        self.twist = Twist2DStamped()
        self.twist.v = 0
        self.twist.omega = 0
        self.now = rospy.get_time()
        
        self.twist_sub = rospy.Subscriber(
            "/cmd_twist",
            Twist2DStamped,
            self.on_twist)
        
        self.pose_pub = rospy.Publisher(
            "/odometry/duck{}".format(THIS_DUCK),
            Odometry,
            queue_size=10)
		
        self.pub_pose()
    
    def pub_pose(self):
        rospy.loginfo("Updated position to:\n{}".format(self.ekf.x))
        msg = Odometry()
        msg.pose.pose.position.x = self.ekf.x[0]
        msg.pose.pose.position.y = self.ekf.x[1]
        msg.pose.pose.orientation.z = math.sin(self.ekf.x[2]/2)
        msg.pose.pose.orientation.w = math.cos(self.ekf.x[2]/2)
        
        self.pose_pub.publish(msg)
    
    def get_time_step(self):
        then, self.now = self.now, rospy.get_time()
        return self.now - then
    
    def on_twist(self, twist):
        self.predict()
        self.twist = twist
        self.pub_pose()
    
    def predict(self):
        self.ekf.predict(self.twist, self.get_time_step())
    
    def on_rssi(self, rssi, them):
        rospy.loginfo("Received {} from duck at\n{}".format(rssi, them))
        self.predict()
        self.ekf.update(np.array([[rssi]]), them)
        self.pub_pose()

if __name__ == '__main__':
    rospy.init_node('pose_filter')
    duck_ids = [int(arg) for arg in rospy.myargv(argv=sys.argv[1:])]
    ekf = make_filter()
    
    me = Me(ekf)
    
    ducks = [Duck(i, me.on_rssi) for i in duck_ids]
    
    rospy.loginfo("Starting up")
    rospy.spin()
