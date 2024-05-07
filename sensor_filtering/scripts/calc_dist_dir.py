#!/usr/bin/env python

import sys
import rospy, roslib
from sensor_msgs.msg import LaserScan
import math
import time

class Filter:

	LINEAR_SPEED = 0.3
	ANGULAR_SPEED = 1.5
	MIN_SCAN_ANGLE = 30 / 180 * math.pi
	MAX_SCAN_ANGLE = 330 / 180 * math.pi
	TIME_ORIENT_CIRCLE = 9
	TIME_POSITION = 4
	TIME_ORIENT = 2 + TIME_ORIENT_CIRCLE
	MIN_DIST_FROM_OBSTACLE = 0.8
	DELTA_TIME_FOR_CIRCLE = 10

	def resetFields(self):
		self.isObstacle = True
		self.isRotation = False
		self.isAfterOddRotation = False
		self.flagTimePosition = False
		self.flagTimeOrient = False
		self.numRotation = 0
		self.startTimePosition = 0
		self.startTimeOrient = 0
		self.lastTimeCircle = 0


	def __init__(self):

		self.isAchieved = False
		self.isSearch = True
		self.prevSearch = self.isSearch
		self.resetFields()

		self.scanSub = rospy.Subscriber("/scan/raw", LaserScan, self.scanCallback, queue_size=1)


	def followerSnakeSearchCallback(self, msg):
		if self.isAchieved: # block
			return
		if msg.is_achieved:
			print("followerSnakeSearchCallback Target is reached!")
			self.stop()
			self.isAchieved = msg.is_achieved
			return
		self.isSearch = msg.is_search
		if self.isSearch != self.prevSearch:
			self.stop()
			self.resetFields()
			self.prevSearch = self.isSearch


	def initStartTimePosition(self):
		if (self.flagTimePosition):
			self.startTimePosition = time.time()
			self.flagTimePosition = False
			# print("startTimePosition=", self.startTimePosition)


	def initStartTimeOrient(self):
		if (self.flagTimeOrient):
			self.startTimeOrient = time.time()
			self.flagTimeOrient = False
			# print("startTimeOrient=", self.startTimeOrient)


	def blockJustCircle(self, isJustCircle):
		if not isJustCircle:
			self.lastTimeCircle = time.time()


	def runRotation(self, isJustCircle):
		self.blockJustCircle(isJustCircle)

		self.initStartTimeOrient()
		curTimeOrient = time.time()
		deltaTimeOrient = curTimeOrient - self.startTimeOrient

		# print("keepTrackOfTimeForSnake curTimeOrient =", curTimeOrient)
		# print("keepTrackOfTimeForSnake deltaTimeOrient =", deltaTimeOrient)
		if isJustCircle:
			time_orient = self.TIME_ORIENT_CIRCLE
		else:
			time_orient = self.TIME_ORIENT
		if deltaTimeOrient >= time_orient:
			# print("keepTrackOfTimeForSnake isRotation")
			self.stop()
			self.lastTimeCircle = time.time()
			self.isRotation = False
			if isJustCircle:
				return
			if (self.numRotation % 2 == 0):
				self.isAfterOddRotation = True
			self.numRotation = (self.numRotation + 1) % 4


	def runAfterOddRotation(self, isJustCircle):
		self.blockJustCircle(isJustCircle)

		self.initStartTimePosition()
		curTimePosition = time.time()
		deltaTimePosition = curTimePosition - self.startTimePosition
		
		# print("keepTrackOfTimeForSnake curTimePosition =", curTimePosition)
		# print("keepTrackOfTimeForSnake deltaTimePosition =", deltaTimePosition)
		if deltaTimePosition >= self.TIME_POSITION:
			# print("keepTrackOfTimeForSnake isAfterOddRotation")
			self.stop()
			self.flagTimeOrient = True
			self.isRotation = True
			self.isAfterOddRotation = False


	def keepTrackOfTimeForSnake(self, isJustCircle=False):
		if (self.isRotation):
			self.runRotation(isJustCircle)
		if (self.isAfterOddRotation):
			self.runAfterOddRotation(isJustCircle)


	def scanCallback(self, scan):
		if self.isAchieved: # block
			return
		isObstacleInFront = False
		minIndex = math.ceil((self.MIN_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)
		maxIndex = math.ceil((self.MAX_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)
		curIndex = minIndex

		# print("SCAN_SCAN_SCAN")
		while curIndex != maxIndex - 1:
			if curIndex < 0:
				curIndex = 360 + curIndex
			if scan.ranges[int(curIndex)] <= self.MIN_DIST_FROM_OBSTACLE:
				isObstacleInFront = True
				break
			curIndex -= 1
		if isObstacleInFront:
			self.isObstacle = True
			if not self.isRotation:
				# print("scanCallback isObstacle")
				self.stop()
				self.isRotation = True
				self.flagTimeOrient = True
				self.isAfterOddRotation = False
				self.flagTimePosition = True
				self.keepTrackOfTimeForSnake() # чтобы избежать возможный повторный круг
		else:
			self.isObstacle = False
		self.pubSnakeSearchStatus(scan.ranges)


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
