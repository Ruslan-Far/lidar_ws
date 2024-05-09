#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
from matplotlib.patches import Arrow

class CalcDistDir:

	STR_DIR = "Направление: "
	STR_DIST = "Расстояние: "
	STR_STATE_STOPPING = "Стоит на месте"
	STR_STATE_LEAVING = "Отдаляется"
	STR_STATE_APPROACHING = "Приближается"
	STR_STATE_UNKNOWN = "Неизвестно"

	SCAN_ANGLE_DEG = 2 # угол = 2 градуса с одной стороны, начиная от центра
	SCAN_ANGLE = SCAN_ANGLE_DEG / 180 * math.pi # перевод в радианы
	N = 10 # количество измерений
	START_DIST = 5.067 # по оси x 5.067 метров от робота до препятствия при запуске программы
	DELTA_DIST = 0.03 # 3 см и меньше - незначительное изменение

	def resetFields(self):
		self.sumDists = 0 # сумма расстояний
		self.numDists = self.N # число измеренных расстояний (не включая неопределенных)
		self.countN = 0 # счетчик замеров


	def __init__(self):
		self.dirForAnim = ""
		self.distForAnim = ""
		self.ultimateDistPrev = self.START_DIST # инициализация начальным значением, которая будет хранить предыдущие расстояния
		self.isDefined = True # переключатель (при неопределенности - значение False)
		self.resetFields()
		self.scanFilteredSub = rospy.Subscriber("/scan/filtered", LaserScan, self.scanFilteredCallback, queue_size=1)


	def calcAverage(self, minIndex, maxIndex, arr):
		curIndex = minIndex
		sumTerms = 0
		numTerms = maxIndex - minIndex + 1 # обзор = 5 градусов

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
			if self.numDists != 0: # если нет неопределенности
				ultimateDist = self.sumDists / self.numDists # окончательное среднее расстояние
				if self.isDefined: # если предыдущее измерение тоже определенное (иначе мы не можем знать направление, так как только вышли из неопределенности)
					if abs(self.ultimateDistPrev - ultimateDist) <= self.DELTA_DIST:
						self.dirForAnim = self.STR_STATE_STOPPING
						print(self.STR_DIR + self.STR_STATE_STOPPING)
					elif self.ultimateDistPrev < ultimateDist:
						self.dirForAnim = self.STR_STATE_LEAVING
						print(self.STR_DIR + self.STR_STATE_LEAVING)
					else:
						self.dirForAnim = self.STR_STATE_APPROACHING
						print(self.STR_DIR + self.STR_STATE_APPROACHING)
				else:
					self.dirForAnim = self.STR_STATE_UNKNOWN
					print(self.STR_DIR + self.STR_STATE_UNKNOWN)
				self.distForAnim = str(ultimateDist)
				print(self.STR_DIST + str(ultimateDist))
				self.ultimateDistPrev = ultimateDist
				self.isDefined = True
			else:
				self.dirForAnim = self.STR_STATE_UNKNOWN
				self.distForAnim = self.STR_STATE_UNKNOWN
				print(self.STR_DIR + self.STR_STATE_UNKNOWN)
				print(self.STR_DIST + self.STR_STATE_UNKNOWN)
				self.isDefined = False
			self.resetFields()


	def start(self):
		rospy.init_node("calc_dist_dir", anonymous = False)
		rate = rospy.Rate(10)
		plt.ion()
		fig, ax = plt.subplots()

		while not rospy.is_shutdown():
			ax.cla()
			ax.set_title("Animation")
			ax.axis([-8, 8, -8, 8])
			ax.set_xlabel("X")
			ax.set_ylabel("Y")
			ax.add_patch(Rectangle((5, -6), 1, 12))
			dx = self.START_DIST - self.ultimateDistPrev
			ax.add_patch(Circle((dx, 0), radius=0.5, color="g"))
			if self.dirForAnim == self.STR_STATE_LEAVING:
				ax.add_patch(Arrow(dx - 0.5, 0, -2, 0, color="r"))
			elif self.dirForAnim == self.STR_STATE_APPROACHING:
				ax.add_patch(Arrow(dx + 0.5, 0, 2, 0, color="r"))
			plt.text(-7, 6, self.STR_DIR + self.dirForAnim)
			plt.text(-7, 4, self.STR_DIST + self.distForAnim)
			plt.draw()
			plt.gcf().canvas.flush_events()
			rate.sleep()


def main(args):
	try:
		calc_dist_dir = CalcDistDir()
		calc_dist_dir.start()
	except rospy.ROSInterruptException:
		print("calc_dist_dir Shutting down")
		plt.ioff()
		plt.show()


if __name__ == '__main__':
	main(sys.argv)
