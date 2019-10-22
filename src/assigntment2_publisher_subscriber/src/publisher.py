#!/usr/bin/env python3
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand


def talker():

    rospy.init_node('talker', anonymous=True)
    steering = rospy.Publisher('/actuators/steering normalized', NormalizedSteeringCommand, queue_size=10)
    speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        steer_command = NormalizedSteeringCommand(value=1.)
        steering.publish(steer_command)

        speed_command = SpeedCommand(value=200.)
        speed.publish(speed_command)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
