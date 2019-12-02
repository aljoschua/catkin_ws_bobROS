#!/usr/bin/env python2

import rospy
import cv_bridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

bridge = cv_bridge.CvBridge()

crop_regions = [
    [(0, 0), (540, 145)],
    [(240, 300), (430, 420)],
    [(210, 340), (430, 500)],
    [(390, 150), (430, 200)],
    [(430, 175), (550, 260)],
    [(530, 240), (570, 280)],
]

# Options
ignore_msgs = 10  # just calibrate for every n messages


def calibrate(image):
    if not image.header.seq % ignore_msgs:
        # convert image message to binary image
        cv_img_orig = bridge.imgmsg_to_cv2(image)
        _, cv_img = cv2.threshold(cv_img_orig, 254, 255, cv2.THRESH_BINARY)

        for crop_region in crop_regions:
            cv2.rectangle(cv_img, *crop_region, color=0, thickness=-1)

        binary.publish(bridge.cv2_to_imgmsg(cv_img))

        # Get all of white pixel coordinates in an array
        tmp = np.where(cv_img == 255)
        points = np.array(zip(tmp[1], tmp[0]))

        # RANSAC parameters from the lecture
        N = 30
        d = 30
        t = 20
        s = 2

        parameters = [(0, 0)]*3
        for line_no in range(3):  # Repeat ransac for all three lines
            inliers = np.array(range(len(points)))

            m = b = 0
            for _ in range(N):

                to_be_fitted = points[np.random.choice(inliers, s)]

                xs = to_be_fitted[:, 0]
                ys = to_be_fitted[:, 1]

                # === Fit Line === https://pythonprogramming.net/how-to-program-best-fit-line-machine-learning-tutorial/
                m = ((np.mean(xs)*np.mean(ys)) - np.mean(xs*ys)) /\
                    ((np.mean(xs)*np.mean(xs)) - np.mean(xs*xs))
                b = np.mean(ys) - m*np.mean(xs)

                new_inliers = []
                for i in range(len(points)):
                    point = points[i]
                    dist_to_line = abs(point[0]*m + -1*point[1] + b)/np.sqrt(m**2 + 1)
                    if dist_to_line < t:
                        new_inliers.append(i)

                if len(new_inliers) < d:
                    inliers = np.array(range(len(points)))
                else:
                    inliers = new_inliers

            parameters[line_no] = m, b

            # Draw line on the original image
            f = lambda x: int(m*x + b)
            cv2.line(cv_img_orig, (0, f(0)), (640, f(640)), color=0, thickness=5)

            # Remove inliers from set of points so future iterations have less search space
            mask = np.ones(len(points), dtype=bool)
            mask[inliers] = False
            points = points[mask]

        lines.publish(bridge.cv2_to_imgmsg(cv_img_orig))

        print parameters


if __name__ == '__main__':
    rospy.init_node("ransac", anonymous=True)
    rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, calibrate)
    binary = rospy.Publisher("/sensors/camera/infra1/image_rect_bin", Image, queue_size=10)
    lines = rospy.Publisher("/sensors/camera/infra1/image_rect_lines", Image, queue_size=10)
    rospy.spin()
