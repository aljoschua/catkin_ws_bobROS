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


def initialize(position):
    global recording
    rospy.sleep(2)
    global x, y, theta, t
    print "received initial position"
    x = position.pose.pose.position.x
    y = position.pose.pose.position.y
    theta = 2 * np.arcsin(position.pose.pose.orientation.w)  # should also consider negative value
    t = position.header.stamp.secs + 1e-9 * position.header.stamp.nsecs
    recording = True
    initial_odometry.unregister()


def store_angle(angle):
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


def publish_odometry(base_msg):
    base_msg.header.stamp = rospy.Time.now()
    base_msg.pose.pose.position.x = x
    base_msg.pose.pose.position.y = y

    base_msg.pose.pose.orientation.w = np.cos(theta/2)
    base_msg.pose.pose.orientation.z = np.sin(theta/2)

    odometry.publish(base_msg)


if __name__ == '__main__':
    rospy.init_node("calibration", anonymous=True)
    rospy.Subscriber("/sensors/steering", SteeringAngle, store_angle)  # Returns angle in radians
    rospy.Subscriber("/sensors/speed", Speed, calibrate)
    initial_odometry = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, initialize)
    odometry = rospy.Publisher("/odometry", Odometry, queue_size=10)


    # Prep Odometry
    base_msg = Odometry()
    base_msg.header.frame_id = "map"
    base_msg.child_frame_id = "base_link"

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        publish_odometry(base_msg)
        rate.sleep()
