#!/usr/bin/python

import rosbag
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import easygui
import numpy.polynomial.polynomial as poly
from array import array
from scipy import interpolate
import os

m1 = []
m2 = []
m3 = []
m4 = []
b1 = []
b2 = []
b3 = []
b4 = []

bagfiles = [f for f in os.listdir('.') if f.endswith('.bag')]
for filename in bagfiles:
	bag = rosbag.Bag(filename)
	M1_speed = []
	M2_speed = []
	M3_speed = []
	M4_speed = []
	
	M1_pwm =[]
	M2_pwm =[]
	M3_pwm =[]
	M4_pwm =[]

	PWM_points = [20000, 40000, 60000]

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m1'):
			M1_speed.append((168000000/msg.values[0])* 2 * np.pi)
			M1_pwm.append(msg.values[1]/65535)

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m2'):
			M2_speed.append((168000000/msg.values[0])* 2 * np.pi)
			M2_pwm.append(msg.values[1]/65535)

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m3'):
			M3_speed.append((168000000/msg.values[0])* 2 * np.pi)
			M3_pwm.append(msg.values[1]/65535)

	for topic, msg, t in bag.read_messages(topics='/crazyflie1/m4'):
			M4_speed.append((168000000/msg.values[0])* 2 * np.pi)
			M4_pwm.append(msg.values[1]/65535)
		
	bag.close()

	# Determine the coeficients
	coefs1 = poly.polyfit(M1_pwm, M1_speed, 1)
	coefs2 = poly.polyfit(M2_pwm, M2_speed, 1)
	coefs3 = poly.polyfit(M3_pwm, M3_speed, 1)
	coefs4 = poly.polyfit(M4_pwm, M4_speed, 1)

	x_new = np.linspace(0, 1, 100)

	speed1 = poly.polyval(x_new, coefs1)
	speed2 = poly.polyval(x_new, coefs2)
	speed3 = poly.polyval(x_new, coefs3)
	speed4 = poly.polyval(x_new, coefs4)

	# Plot interpolated functions
	plt.plot(x_new, speed1)
	plt.plot(x_new, speed2)
	plt.plot(x_new, speed3)
	plt.plot(x_new, speed4)

	m1.append(coefs1[1])
	m2.append(coefs2[1])
	m3.append(coefs3[1])
	m4.append(coefs4[1])
	b1.append(coefs1[0])
	b2.append(coefs2[0])
	b3.append(coefs3[0])
	b4.append(coefs4[0])

b1_dt = np.diff(b1)
b2_dt = np.diff(b2)
b3_dt = np.diff(b3)
b4_dt = np.diff(b4)

m1_dt = np.diff(m1)
m2_dt = np.diff(m2)
m3_dt = np.diff(m3)
m4_dt = np.diff(m4)

b1_std = np.std(b1_dt)
b2_std = np.std(b2_dt)
b3_std = np.std(b3_dt)
b4_std = np.std(b4_dt)

m1_std = np.std(m1_dt)
m2_std = np.std(m2_dt)
m3_std = np.std(m3_dt)
m4_std = np.std(m4_dt)

print("-------------------------------------------")
print("Moror 1:")
print("Std of m: " + str(m1_std/(30*1000)))
print("Std of b: " + str(b1_std/(30*1000)))
print("-------------------------------------------")
print("Moror 2:")
print("Std of m: " + str(m2_std/(30*1000)))
print("Std of b: " + str(b2_std/(30*1000)))
print("-------------------------------------------")
print("Moror 3:")
print("Std of m: " + str(m3_std/(30*1000)))
print("Std of b: " + str(b3_std/(30*1000)))
print("-------------------------------------------")
print("Moror 4:")
print("Std of m: " + str(m4_std/(30*1000)))
print("Std of b: " + str(b4_std/(30*1000)))

print("-------------------------------------------")
print("Average Std of m: " + str(((m1_std+m2_std+m3_std+m4_std)/4)/(30*1000)))
print("Average Std of b: " + str(((b1_std+b2_std+b3_std+b4_std)/4)/(30*1000)))

plt.show()
