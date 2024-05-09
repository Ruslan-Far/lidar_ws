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

		self.ultimateDistPrev = self.START_DIST # инициализация начальным значением, которая будет хранить предыдущие расстояния

		self.isDefined = True # переключатель (при неопределенности значение False)

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

		# middle = round(((abs(scan.angle_min) + abs(scan.angle_max)) / 2) / scan.angle_increment)
		# scanAngle = round(self.SCAN_ANGLE / scan.angle_increment)
		# minIndex = middle - scanAngle
		# maxIndex = middle + scanAngle

		# curDist = self.calcAverage(minIndex, maxIndex, scan.ranges) # текущее среднее расстояние (из обзора в 5 градусов) в одном измерении
		# if curDist != -1:
		# 	self.sumDists += curDist
		# else:
		# 	self.numDists -= 1
		# self.countN += 1

		# if self.countN == self.N: # когда проведено 10 измерений
		# 	if self.numDists != 0: # если нет неопределенности
		# 		ultimateDist = self.sumDists / self.numDists # окончательное среднее расстояние
		# 		if self.isDefined: # если предыдущее измерение тоже определенное (иначе мы не можем знать направление, так как только вышли из неопределенности)
		# 			if abs(self.ultimateDistPrev - ultimateDist) <= self.DELTA_DIST:
		# 				print("Стоит на месте")
		# 			elif self.ultimateDistPrev < ultimateDist:
		# 				print("Отдаляется")
		# 			else:
		# 				print("Приближается")
		# 		print("Расстояние =", ultimateDist)
		# 		self.ultimateDistPrev = ultimateDist
		# 		self.isDefined = True
		# 	else:
		# 		print("Неизвестно")
		# 		self.isDefined = False
		# 	self.drawAnimation("Стоит на месте", str(ultimateDist))
		# 	self.resetFields()

		print("scan")
		# self.myanimate(scan.ranges[0])


	def myanimate(self, n):
		plt.clf()
		# self.fig, self.ax = plt.subplots()
		# self.ax.set_title("Animation")
		# self.ax.axis([-8, 8, -8, 8])
		# self.ax.set_xlabel("X")
		# self.ax.set_ylabel("Y")
		# dir = "Стоит на месте"
		self.ax.add_patch(Rectangle((5, -6), n, 12))
		plt.draw()
		plt.gcf().canvas.flush_events()
		print("aaaaaaaaaaaaaannnnnnnnnnnnnnnnnnnnnnnnnniiiiiiiiiiiiiiiiiiiiiiiiimmmmmmmmmmmmmmmmmmmmmmmeeeeeeeeeee")


	def drawAnimation(self, dir, dist):
		plt.ion()
		fig, ax = plt.subplots()
		ax.set_title("Animation")
		ax.axis([-8, 8, -8, 8])
		ax.set_xlabel("X")
		ax.set_ylabel("Y")
		# ax.grid()

		plt.clf()
		ax.add_patch(Rectangle((5, -6), 1, 12))
		ax.add_patch(Circle((0, 0), 0.5))
		ax.add_patch(Arrow(1, 0, 2, 0))
		# dir = "Направление: "
		# dir += "Стоит на месте"
		# dist = "Расстояние = "
		# n = 10
		# dist += str(n)
		# plt.text(-6, 6, s, horizontalalignment="center")
		plt.text(-7, 6, "Направление: " + dir)
		plt.text(-7, 4, "Расстояние: " + dist)
		print("aaaaaaaaaaaaaannnnnnnnnnnnnnnnnnnnnnnnnniiiiiiiiiiiiiiiiiiiiiiiiimmmmmmmmmmmmmmmmmmmmmmmeeeeeeeeeee")

		# ax.clear()
		# plt.show()
		plt.draw()
		plt.gcf().canvas.flush_events()
		# plt.ioff()
		# plt.show()


	def gaussian(self, x, delay, sigma):
		'''
		Функция, график которой будет отображаться процессе анимации
		'''
		return np.exp(-((x - delay) / sigma) ** 2)

	def start(self):
		rospy.init_node("calc_dist_dir", anonymous = False)
		rate = rospy.Rate(10)
		maxSize = 200
		sigma = 10.0
		x = np.arange(maxSize)
		y = np.zeros(maxSize)
		plt.ion()
		# plt.clf()
		fig, ax = plt.subplots()
		ax.add_patch(Circle((0, 0), 0.5))
		# ax.set_title("Animation")
		# ax.axis([-8, 8, -8, 8])
		# ax.set_xlabel("X")
		# ax.set_ylabel("Y")
		# ax.add_patch(Rectangle((5, -6), 1, 12))
		# plt.draw()
		# plt.gcf().canvas.flush_events()
		


		for delay in np.arange(-50.0, 200.0, 1.0):
			# y = self.gaussian(x, delay, sigma)

			# plt.clf()
			# fig, ax = plt.subplots()
			# ax.clear()

			ax.cla()
			ax.set_title("Animation")
			ax.axis([-8, 8, -8, 8])
			ax.set_xlabel("X")
			ax.set_ylabel("Y")
			ax.add_patch(Rectangle((5, -6), 1, abs(delay / 10)))
			plt.text(-7, 6, "Направление: " + str(delay))


			# plt.plot(x, y)

			# plt.xlim(0, maxSize)
			# plt.ylim(-1.1, 1.1)

			plt.draw()
			plt.gcf().canvas.flush_events()
			rate.sleep()
		# while not rospy.is_shutdown():
		# 	plt.clf()

		# 	plt.text(-7, 6, "Направление: ")
			
		# 	plt.draw()
		# 	plt.gcf().canvas.flush_events()
		# 	rate.sleep()
		


		# ani = FuncAnimation(self.fig, func=self.myanimate, interval=1000)

		# plt.show()

		# rospy.spin()


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
