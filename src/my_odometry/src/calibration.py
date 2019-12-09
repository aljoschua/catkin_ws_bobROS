#!/usr/bin/env python2

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from autominy_msgs.msg import Speed, SteeringAngle

# Constants
l = .27  # Distance between front and rear axle in meters

# Variables described in the assignment
x = 0
y = 0
phi = 0  # local angle (steering sensor)
theta = 0  # global angle
t = 0

total_distance = 0
recording = False
initial_odometry = None


def initialize(position):  # Will this be called again after unsubscribing ?
    global recording
    if not recording:
        global x, y, theta, t
        print "received initial position"
        x = position.pose.pose.position.x
        y = position.pose.pose.position.y
        theta = 2 * np.arcsin(position.pose.pose.orientation.w)  # should also consider negative value
        t = position.header.stamp.secs + 1e-9 * position.header.stamp.nsecs
        recording = True
        initial_odometry.unregister()


def storeAngle(angle):
    global phi
    phi = angle.value


def calibrate(speed):
    if recording:
        global x, y, theta, t
        global total_distance
        new_timestamp = speed.header.stamp.secs + 1e-9 * speed.header.stamp.nsecs
        delta_t = new_timestamp - t
        t = new_timestamp

        x_dot = speed.value * np.cos(theta)
        y_dot = speed.value * np.sin(theta)
        theta_dot = speed.value/l * np.tan(phi)

        delta_x = delta_t * x_dot
        delta_y = delta_t * y_dot
        x += delta_x
        y += delta_y
        theta += delta_t * theta_dot

        total_distance += np.sqrt(delta_x**2 + delta_y**2)
        print total_distance


if __name__ == '__main__':
    rospy.init_node("calibration", anonymous=True)
    rospy.Subscriber("/sensors/steering", SteeringAngle, storeAngle)  # Returns angle in radians
    rospy.Subscriber("/sensors/speed", Speed, calibrate)
    initial_odometry = rospy.Subscriber("/communication/gps/16", Odometry, initialize)
    odometry = rospy.Publisher("/odometry", Odometry, queue_size=10)
    rospy.spin()
