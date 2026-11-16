#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


COLOR_RANGES = {
    "red": ((0, 0, 150), (90, 90, 255)),
    "green": ((0, 150, 0), (90, 255, 90)),
    "blue": ((150, 0, 0), (255, 90, 90)),
}


class ColorVision:
    def __init__(self):
        rospy.init_node("vision_mission")
        self.bridge = CvBridge()
        self.last_color = None
        rospy.Subscriber("/usb_cam/image_raw", Image,
                         self.image_cb, queue_size=1)

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        color = self.detect_color(frame)

        if color != self.last_color:
            rospy.loginfo("检测到颜色：%s", color)
            self.last_color = color

    def detect_color(self, frame):
        best_name = None
        best_area = 0

        for name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(frame, np.array(lower), np.array(upper))
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area = cv2.contourArea(c)
                if area > 500 and area > best_area:
                    best_area = area
                    best_name = name

        return best_name


if __name__ == "__main__":
    ColorVision()
    rospy.spin()