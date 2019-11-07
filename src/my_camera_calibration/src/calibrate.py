#!/usr/bin/env python2

import rospy
import cv_bridge
import cv2
import numpy
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt


# Region definition
region_start_points = [(190, 220),  # start points for regions for the markers to be in
                       (480, 210),  # this list is also in the same order as objectPoints
                       (230, 150),
                       (430, 140),
                       (260, 110),
                       (410, 100)]
region_rect_size = 40  # rectangle should be a square with this side length
region_end_points = map(lambda tup: tuple(map(lambda x: x + region_rect_size,
                                              tup)),
                        region_start_points)


# Intrinsic parameter and marker coordinates definition
objectPoints = numpy.array([
    [.5, .2, 0],
    [.5, -.2, 0],
    [.8, .2, 0],
    [.8, -.2, 0],
    [1.1, .2, 0],
    [1.1, -.2, 0]
])
cameraMatrix = numpy.array([
    [383.794464111328, 0.0, 322.3056945800781],
    [0, 383.794464111328, 241.67051696777344],
    [0, 0, 1]
])
distCoeffs = numpy.zeros((5, 1))

# Options
ignore_msgs = 5  # just calibrate for every n messages
debug = False  # draws rectangles and shows the binary image if set to True


def find_center_for_region(image, start_point, end_point):
    """finds the center of a region (given by the upper left and lower right point of a rectangle),
    by averaging all x and y values where their pixel is white"""
    x_occurrences = []
    y_occurrences = []
    for x in range(start_point[0], end_point[0]):
        for y in range(start_point[1], end_point[1]):
            if image[y][x] == 255:
                x_occurrences.append(x)
                y_occurrences.append(y)

    return numpy.mean(x_occurrences), numpy.mean(y_occurrences)


def calibrate(image):
    if not image.header.seq % ignore_msgs:
        # convert image message to binary image
        array = cv_bridge.CvBridge().imgmsg_to_cv2(image)
        _, cv_img = cv2.threshold(array, 254, 255, cv2.THRESH_BINARY)

        # calculate region center for every region
        region_centers = []
        for i in range(len(region_start_points)):
            start_point = region_start_points[i]
            end_point = region_end_points[i]

            if debug:
                cv2.rectangle(cv_img, start_point, end_point, (255, 255, 255), thickness=1)

            center = find_center_for_region(cv_img, start_point, end_point)
            region_centers.append(center)

        _,  rvec, tvec = cv2.solvePnP(objectPoints, numpy.array(region_centers), cameraMatrix, distCoeffs)
        print "rvec:\n", rvec, "\ntvec:\n", tvec

        # Create transformation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        tmp = numpy.hstack((rotation_matrix, tvec))
        ht_matrix = numpy.vstack((tmp, [0, 0, 0, 1]))

        print "homogeneous transformation matrix:\n", ht_matrix
        print "inverse of h.t. matrix:\n", numpy.linalg.inv(ht_matrix)

        if debug:
            plt.imshow(cv_img, "gray")
            plt.show()


if __name__ == '__main__':
    rospy.init_node('calibrator', anonymous=True)
    rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, calibrate)
    rospy.spin()
