#! /usr/bin/env python
# -*- coding:utf-8

import rospy
import cv2, cv_bridge
import numpy as np
import matplotlib.pyplot as plt
import roslib
import sys
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool


MIN_MATCH_COUNT = 25
show_matched_points = True


class image_converter:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.surf = cv2.xfeatures2d.SURF_create(1000)
        self.stop_sign_img = cv2.imread('parking.png', cv2.IMREAD_COLOR)
        self.match_pub = rospy.Publisher("matches/is_block", Bool)
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)
        self.match = False

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        cv2.useOptimized()
        cv2.setUseOptimized(True)

        kp1, des1 = self.surf.detectAndCompute(self.stop_sign_img, None)
        kp2, des2 = self.surf.detectAndCompute(imageGray, None)

        FLANN_INDEX_KDTREE = 1

        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        matches = None

        try:
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1, des2, k=2)

        except Exception as ex:
            print('knnMatch error')
            return

        good = []

        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)

        outer_dst_pts = np.float32([])

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            outer_dst_pts = dst_pts

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            matchesMask = mask.ravel().tolist()

            h, w, d = self.stop_sign_img.shape

            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

            dst = None

            try:
                dst = cv2.perspectiveTransform(pts, M)
            except Exception as ex:
                print("perspectievTransform error : dst = %s" % dst)

                return

            image = cv2.polylines(image, [np.int32(dst)], True, (255, 0, 0), 3, cv2.LINE_AA)

            self.match = True

            rospy.logdebug('Parking Sign detected : %s' % self.match)


        else:
            self.match = False
            rospy.logdebug("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))

            matchesMask = None

        draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=matchesMask, flags=2)

        matches_img = cv2.drawMatches(self.stop_sign_img, kp1, image, kp2, good, None, **draw_params)

        if show_matched_points:
            for pt in outer_dst_pts:
                x, y = pt[0]

                cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

        self.match_pub.publish(self.match)

        if self.match:
            time.sleep(5)

        cv2.imshow('parking_match', matches_img)
        # cv2.imshow("parking", image)
        cv2.waitKey(3)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True, log_level=rospy.DEBUG)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
