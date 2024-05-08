#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import LaserScan
import math

class CalcDistDir:

	SCAN_ANGLE_DEG = 2 # угол = 2 градуса с одной стороны, начиная от центра
	SCAN_ANGLE = SCAN_ANGLE_DEG / 180 * math.pi # перевод в радианы
	NUM_TERMS = 2 * SCAN_ANGLE_DEG + 1 # в итоге получится обзор = 2 * 2 + 1 = 5 градусов

	N = 10 # количество измерений
	START_DIST = 5 # по оси x 5 метров от робота до препятствия при запуске программы

	def resetFields(self):
		self.sumDists = 0 # сумма расстояний
		self.numDists = self.N # число измеренных расстояний (не включая неопределенных)
		self.countN = 0 # счетчик замеров


	def __init__(self):

		self.ultimateDistPrev = self.START_DIST # инициализация начальным значением, которая будет хранить предыдущие расстояния

		self.isDefined = True # переключатель (при неопределенности значение False)

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

		curDist = self.calcAverage(minIndex, maxIndex, scan.ranges) # текущее среднее расстояние (из обзора в 5 градусов) в одном измерении
		if curDist != -1:
			self.sumDists += curDist
		else:
			self.numDists -= 1
		self.countN += 1

		if self.countN == self.N: # когда проведено 10 измерений
			ultimateDist = self.sumDists / self.N # окончательное среднее расстояние
			if ultimateDist != 0: # если нет неопределенности
				if self.isDefined: # если предыдущее измерение тоже определенное (иначе мы не можем знать направление, так как только вышли из неопределенности)
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
