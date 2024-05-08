#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Глобальные переменные для хранения данных
data_x = []
data_y = []

def callback(msg):
    global data_x, data_y
    data_x.append(rospy.Time.now().to_sec())  # Добавляем текущее время в секундах
    data_y.append(msg.data)

def animate(i):
    plt.cla()  # Очищаем предыдущий график
    plt.plot(data_x, data_y)

def main():
    rospy.init_node('exper', anonymous=True)
    rospy.Subscriber("/scan/filtered", Float32, callback)

    fig = plt.figure()
    ani = FuncAnimation(fig, animate, interval=1000)  # Обновляем каждую секунду

    plt.show()

if __name__ == '__main__':
    main()