#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SteeringFeedback, NormalizedSpeedCommand, NormalizedSteeringCommand, Tick

import math

car_gps_id = 9  # for 129
car_gps_id = 11  # for 121
#calibrate_for = 1.0  # left
calibrate_for = 0.0  # straight
#calibrate_for = -1.0  # right

distance_travelled = 0
last_position = (-1, -1)

count_ticks = False
count_distance = False
ticks = 0


def publish(publisher, command):
    for i in range(10):
        publisher.publish(command)
        rospy.sleep(0.1)


def save_position(odometry):
    global last_position, count_distance, distance_travelled
    current = (odometry.pose.pose.position.x, odometry.pose.pose.position.y)
    if last_position != (-1, -1) and count_distance:
        delta = math.sqrt(abs(last_position[0]-current[0])**2 + abs(last_position[0]-current[0])**2)
        if delta > 0.001:
            distance_travelled += delta
    last_position = current


def save_ticks(tick):
    global count_ticks, ticks
    if count_ticks:
        ticks += tick.value


def main():
    global count_ticks, count_distance, distance_travelled
    rospy.sleep(1)  # wait for node to fully initialize
    publish(steering, NormalizedSteeringCommand(value=calibrate_for))
    print "Calibrator started, waiting for gps signal..."
    while last_position == (-1, -1):
        pass
    publish(speed, NormalizedSpeedCommand(value=0.1))
    count_ticks = count_distance = True
    rospy.sleep(6)
    count_ticks = count_distance = False
    publish(speed, NormalizedSpeedCommand(value=0.0))

    print "Finished. Measured distance:", distance_travelled, "; measured ticks:", ticks
    print "Calculated distance tick ratio: ", distance_travelled/ticks


if __name__ == '__main__':
    rospy.init_node('calibrator', anonymous=True)
    rospy.Subscriber("/communication/gps/" + str(car_gps_id), Odometry, save_position)
    speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand, queue_size=10)
    steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    rospy.Subscriber("/sensors/arduino/ticks", Tick, save_ticks)

    main()
