#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SteeringFeedback, NormalizedSpeedCommand, NormalizedSteeringCommand

import math

car_gps_id = 9  # for 129
#calibrate_for = 1.0  # left
#calibrate_for = 0.0  # straight
calibrate_for = -1.0  # right

distance_travelled = 0
last_position = (-1, -1)

count_ticks = False
ticks = 0


def save_position(odometry):
    global last_position, distance_travelled
    current = (odometry.pose.pose.position.x, odometry.pose.pose.position.y)
    if last_position != (-1, -1):
        distance_travelled += math.sqrt(abs(last_position[0]-current[0])**2 + abs(last_position[0]-current[0])**2)
    last_position = current


def save_ticks(data):
    global count_ticks, ticks
    if count_ticks:
        ticks += 1


def main():
    global count_ticks, distance_travelled
    steering.publish(NormalizedSteeringCommand(value=calibrate_for))
    while last_position == (-1, -1):
        pass
    count_ticks = True
    speed.publish(NormalizedSpeedCommand(value=.2))
    rospy.sleep(5)  # drive approximately 1 meter
    count_ticks = False
    speed.publish(NormalizedSpeedCommand(value=0))

    print(distance_travelled/ticks)


if __name__ == '__main__':
    rospy.init_node('calibrator', anonymous=True)
    rospy.Subscriber("/communication/gps/" + str(car_gps_id), Odometry, save_position)
    speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand, queue_size=10)
    steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    rospy.Subscriber("/sensors/arduino/ticks", None, save_ticks)

    main()
