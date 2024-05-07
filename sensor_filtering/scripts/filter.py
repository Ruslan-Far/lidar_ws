#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class Filter:

	def __init__(self):
		self.scanRawSub = rospy.Subscriber("/scan/raw", LaserScan, self.scanRawCallback, queue_size=1)
		self.scanFilteredPub = rospy.Publisher("/scan/filtered", LaserScan, queue_size=1)


	def pubLaserScan(self, scan):
		self.scanFilteredPub.publish(scan)


	def scanRawCallback(self, scan):
		minIndex = 0
		maxIndex = len(scan.ranges) - 1
		curIndex = minIndex
		scanFiltered = LaserScan()

		scanFiltered.header.seq = scan.header.seq
		scanFiltered.header.stamp.secs = scan.header.stamp.secs
		scanFiltered.header.stamp.nsecs = scan.header.stamp.nsecs
		scanFiltered.header.frame_id = scan.header.frame_id

		scanFiltered.angle_min = scan.angle_min
		scanFiltered.angle_max = scan.angle_max
		scanFiltered.angle_increment = scan.angle_increment
		scanFiltered.time_increment = scan.time_increment
		scanFiltered.scan_time = scan.scan_time

		scanFiltered.range_min = scan.range_min
		scanFiltered.range_max = scan.range_max

		scanFiltered.ranges = np.array(scan.ranges).copy()
		scanFiltered.intensities = np.array(scan.intensities).copy()

		while curIndex <= maxIndex:
			if not (scan.ranges[curIndex] >= scan.range_min and scan.ranges[curIndex] <= scan.range_max):
				scanFiltered.ranges[curIndex] = -1
			curIndex += 1
		self.pubLaserScan(scanFiltered)


	def start(self):
		rospy.init_node("filter", anonymous = False)
		rospy.spin()


def main(args):
	try:
		filter = Filter()
		filter.start()
	except rospy.ROSInterruptException:
		print("filter Shutting down")


if __name__ == '__main__':
	main(sys.argv)
