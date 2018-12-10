#!/usr/bin/env python
	
import matplotlib.pyplot as plt
import numpy as np
import random
import math
data = [[] for i in range(10)]

with open('flight.log','r') as f:
	time = 0
	for line in f:
		try:
			if not("no pose" in line):
				pose = [ float(sub) for sub in line.split()]
				for i in range(1, 10):
					data[i].append(pose[i - 1])
			else:
				for i in range(1, 10):
					data[i].append(0)

			data[0].append(time)
			time += 1
		except:
			pass			
		
N = time
N_min = 0
N_max = N

for i in range(10):
	if len(data[i]) > len(data[0]):
		data[i].pop()
	data[i] = np.array(data[i])

minor_ticks = []
for i in range(N_min, N_max):
	minor_ticks.append(i)
major_ticks = []
for i in range((N_max - N_min)//10):
	major_ticks.append(N_min + i*10)

plot_type = 'all'

if plot_type == 'all':
	fig, ((aix, aiy, aiz), (ax, ay, az), (avx, avy, avz)) = plt.subplots(figsize=(30, 10), nrows=3, ncols=3)
	aix.plot(data[0], data[1])
	aix.set_xticks(minor_ticks, minor=True)
	aix.set_xticks(major_ticks, minor=False)
	aix.grid(which='minor')
	aix.grid(which='major')
	aiy.plot(data[0], data[2])
	aiy.set_xticks(minor_ticks, minor=True)
	aiy.set_xticks(major_ticks, minor=False)
	aiy.grid(which='minor')
	aiy.grid(which='major')
	aiz.plot(data[0], data[3])
	aiz.set_xticks(minor_ticks, minor=True)
	aiz.set_xticks(major_ticks, minor=False)
	aiz.grid(which='minor')
	aiz.grid(which='major')
	ax.plot(data[0], data[4])
	ax.set_xticks(minor_ticks, minor=True)
	ax.set_xticks(major_ticks, minor=False)
	ax.grid(which='minor')
	ax.grid(which='major')
	ay.plot(data[0], data[5])
	ay.set_xticks(minor_ticks, minor=True)
	ay.set_xticks(major_ticks, minor=False)
	ay.grid(which='minor')
	ay.grid(which='major')
	az.plot(data[0], data[6])
	az.set_xticks(minor_ticks, minor=True)
	az.set_xticks(major_ticks, minor=False)
	az.grid(which='minor')
	az.grid(which='major')
	avx.plot(data[0], data[7])
	avx.set_xticks(minor_ticks, minor=True)
	avx.set_xticks(major_ticks, minor=False)
	avx.grid(which='minor')
	avx.grid(which='major')
	avy.plot(data[0], data[8])
	avy.set_xticks(minor_ticks, minor=True)
	avy.set_xticks(major_ticks, minor=False)
	avy.grid(which='minor')
	avy.grid(which='major')
	avz.plot(data[0], data[9])
	avz.set_xticks(minor_ticks, minor=True)
	avz.set_xticks(major_ticks, minor=False)
	avz.grid(which='minor')
	avz.grid(which='major')

elif plot_type == 'x':
	fig, ((aix), (ax), (avx)) = plt.subplots(figsize=(30, 10), nrows=3, ncols=1)
	aix.plot(data[0][N_min:N_max], data[1][N_min:N_max])
	aix.set_xticks(minor_ticks, minor=True)
	aix.set_xticks(major_ticks, minor=False)
	aix.grid(which='minor')
	aix.grid(which='major')
	ax.plot(data[0][N_min:N_max], data[4][N_min:N_max])
	ax.set_xticks(minor_ticks, minor=True)
	ax.set_xticks(major_ticks, minor=False)
	ax.grid(which='minor')
	ax.grid(which='major')
	avx.plot(data[0][N_min:N_max], data[7][N_min:N_max])
	avx.set_xticks(minor_ticks, minor=True)
	avx.set_xticks(major_ticks, minor=False)
	avx.grid(which='minor')
	avx.grid(which='major')

elif plot_type == 'y':
	fig, ((aiy), (ay), (avy)) = plt.subplots(figsize=(30, 10), nrows=3, ncols=1)
	aiy.plot(data[0][N_min:N_max], data[2][N_min:N_max])
	aiy.set_xticks(minor_ticks, minor=True)
	aiy.set_xticks(major_ticks, minor=False)
	aiy.grid(which='minor')
	aiy.grid(which='major')
	ay.plot(data[0][N_min:N_max], data[5][N_min:N_max])
	ay.set_xticks(minor_ticks, minor=True)
	ay.set_xticks(major_ticks, minor=False)
	ay.grid(which='minor')
	ay.grid(which='major')
	avy.plot(data[0][N_min:N_max], data[8][N_min:N_max])
	avy.set_xticks(minor_ticks, minor=True)
	avy.set_xticks(major_ticks, minor=False)
	avy.grid(which='minor')
	avy.grid(which='major')

elif plot_type == 'z':
	fig, ((aiz), (az), (avz)) = plt.subplots(figsize=(30, 10), nrows=3, ncols=1)
	aiz.plot(data[0][N_min:N_max], data[3][N_min:N_max])
	aiz.set_xticks(minor_ticks, minor=True)
	aiz.set_xticks(major_ticks, minor=False)
	aiz.grid(which='minor')
	aiz.grid(which='major')
	az.plot(data[0][N_min:N_max], data[6][N_min:N_max])
	az.set_xticks(minor_ticks, minor=True)
	az.set_xticks(major_ticks, minor=False)
	az.grid(which='minor')
	az.grid(which='major')
	avz.plot(data[0][N_min:N_max], data[9][N_min:N_max])
	avz.set_xticks(minor_ticks, minor=True)
	avz.set_xticks(major_ticks, minor=False)
	avz.grid(which='minor')
	avz.grid(which='major')

elif plot_type == 'sp':
	fig, ((x, y, z), (sx, sy, sz)) = plt.subplots(figsize=(30, 10), nrows=2, ncols=3)
	x.plot(data[0], data[1])
	y.plot(data[0], data[2])
	z.plot(data[0], data[3])
	sx.plot(data[0], data[4])
	sy.plot(data[0], data[5])
	sz.plot(data[0], data[6])


fig.tight_layout()
plt.show()