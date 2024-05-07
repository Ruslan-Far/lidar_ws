#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np

class CalcDistDir:

	SCAN_ANGLE_DEG = 2 # угол = 2 градуса с одной стороны, начиная от центра
	SCAN_ANGLE = SCAN_ANGLE_DEG / 180 * math.pi # перевод в радианы
	NUM_TERMS = 2 * SCAN_ANGLE_DEG + 1 # в итоге получится обзор = 2 * 2 + 1 = 5 градусов

	N = 10 # количество измерений
	START_DIST = 5 # по оси x 5 метров от робота до препятствия при запуске программы

	def resetFields(self):
		self.sumDists = 0
		self.numDists = self.N
		self.countN = 0
		# self.buf = np.zeros((self.N))


	def __init__(self):

		self.ultimateDistPrev = self.START_DIST

		self.isDefined = True

		self.resetFields()

		self.scanFilteredSub = rospy.Subscriber("/scan/filtered", LaserScan, self.scanFilteredCallback, queue_size=1)


	def calcAverage(self, minIndex, maxIndex, arr):
		curIndex = minIndex
		sumTerms = 0
		numTerms = self.NUM_TERMS

		while curIndex <= maxIndex:
			term = arr[curIndex]
			if term != -1:
				sumTerms += term
			else:
				numTerms -= 1
			curIndex += 1

		if numTerms != 0:
			return sumTerms / numTerms
		return -1


	def scanFilteredCallback(self, scan):
		middle = round(((abs(scan.angle_min) + abs(scan.angle_max)) / 2) / scan.angle_increment)
		scanAngle = round(self.SCAN_ANGLE / scan.angle_increment)
		minIndex = middle - scanAngle
		maxIndex = middle + scanAngle
		# curIndex = minIndex
		# sumTerms = 0
		# numTerms = self.NUM_TERMS




		# while curIndex <= maxIndex:
		# 	term = scan.ranges[curIndex]
		# 	if term != -1:
		# 		sumTerms += term
		# 	else:
		# 		numTerms -= 1
		# 	curIndex += 1

		# self.buf[self.countN] = self.calcAverage(minIndex, maxIndex, scan.ranges, self.NUM_TERMS)
		# self.countN += 1
		# if self.countN >= self.N:
		# 	self.countN = 0

		# if self.buf[self.N - 1] == 0:
		# 	return
		# ultimateDist = self.calcAverage(0, self.N - 1, self.buf, self.N)
		# sumTerms = 0
		# curIndex = 0





		curDist = self.calcAverage(minIndex, maxIndex, scan.ranges)
		if curDist != -1:
			self.sumDists += curDist
		else:
			self.numDists -= 1
		self.countN += 1

		if self.countN == self.N:
			ultimateDist = self.sumDists / self.N
			if ultimateDist != 0:
				if self.isDefined:
					if self.ultimateDistPrev == ultimateDist:
						print("Стоит на месте")
					elif self.ultimateDistPrev < ultimateDist:
						print("Отдаляется")
					else:
						print("Приближается")
				print("Расстояние =", ultimateDist)
				self.ultimateDistPrev = ultimateDist
				self.isDefined = True
			else:
				print("Неизвестно")
				self.isDefined = False
			self.resetFields()


	def start(self):
		rospy.init_node("calc_dist_dir", anonymous = False)
		rospy.spin()


def main(args):
	try:
		calc_dist_dir = CalcDistDir()
		calc_dist_dir.start()
	except rospy.ROSInterruptException:
		print("calc_dist_dir Shutting down")


if __name__ == '__main__':
	main(sys.argv)
