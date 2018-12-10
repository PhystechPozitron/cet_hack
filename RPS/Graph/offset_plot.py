#!/usr/bin/env python
	
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
data = [[] for i in range(7)]

filename = sys.argv[1] 
with open(filename,'r') as f:
	N = 0
	for line in f:
		try:
			pose = [ float(sub) for sub in line.split()]
			for i in range(1, 7):
				data[i].append(pose[i - 1])
			data[0].append(N)
			N += 1
		except:
			pass
			
for i in range(7):
	if len(data[i]) > len(data[0]):
		data[i].pop()
	data[i] = np.array(data[i])

mean = [0, 0, 0]
time = []
offset = []
square_sum = 0
N_min = 200
N_max = N-50
for j in range(N_min, N_max):
	time.append((j - N_min)/20.)
	offset.append(math.sqrt(data[4][j]**2 + data[5][j]**2 + data[6][j]**2))
	square_sum += data[4][j]**2 + data[5][j]**2 + data[6][j]**2 

res_mean = math.sqrt((square_sum)/(N_max - N_min))
res_max = max(offset)

time = np.array(time) 
offset = np.array(offset)
print('mean = {0:.3f}, max = {1:.3f}'.format(res_mean, res_max))

fig, graph = plt.subplots(figsize=(7, 5), nrows=1, ncols=1)
graph.plot(time, offset)
t_max = int(max(time))

minor_x = np.array([i/2. for i in range(2*t_max)])
minor_y = np.array([i/100. for i in range(30)]) 
graph.set_xlim(xmin=0, xmax=t_max)
graph.set_ylim(ymin=0, ymax=0.3)
graph.set_xticks(minor_x, minor=True)
graph.set_yticks(minor_y, minor=True)
graph.grid(which="minor", linestyle=":")
graph.grid(which="major", linestyle="-")
graph.set_xlabel("t, c")
graph.set_ylabel("offset, m")

fig.tight_layout()
plt.show()