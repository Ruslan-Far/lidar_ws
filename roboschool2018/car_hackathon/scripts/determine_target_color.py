#!/usr/bin/env python
#coding=utf-8

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Final

ROS_NODE_NAME: Final[str] = "subscriber"
ROS_IMAGE_TOPIC: Final[str] = "/car_gazebo/camera1/image_raw"
WIDTH: Final[int] = 1280
HEIGHT: Final[int] = 720
WINDOW_ORIG: Final[str] = "original"
WINDOW_BIN: Final[str] = "binary"
TRACK_H_MIN: Final[str] = "h_min"
TRACK_H_MAX: Final[str] = "h_max"
TRACK_S_MIN: Final[str] = "s_min"
TRACK_S_MAX: Final[str] = "s_max"
TRACK_V_MIN: Final[str] = "v_min"
TRACK_V_MAX: Final[str] = "v_max"

h_min = 115
h_max = 125
s_min = 0
s_max = 255
v_min = 0
v_max = 255

def nothing(arg):
	pass


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
	# img_bgr = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
	img_bgr = cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
	img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
	img_rgb = cv2.resize(img_rgb, (WIDTH, HEIGHT))

	img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

	img_hsv_thresh = cv2.inRange(img_hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
	# img_hsv_thresh = 255 - img_hsv_thresh

	cv2.imshow(WINDOW_ORIG, img_rgb)
	# cv2.imshow(WINDOW_BIN, img_hsv_thresh)
	cv2.waitKey(1)


def main() -> None:
	rospy.init_node(ROS_NODE_NAME)

	sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout = 3.0)

	if sample is not None:
		rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

	cv_bridge: CvBridge = CvBridge()

	rospy.Subscriber(ROS_IMAGE_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge), queue_size = None)

	rospy.spin()


if __name__ == '__main__':
	main()
