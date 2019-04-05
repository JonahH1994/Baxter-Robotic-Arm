#!/usr/bin/env python
# coding=utf8
#import matplotlib
#matplotlib.use('Agg')
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import rospy
from baxter_gazebo.srv import OccupancyGridPlot as ogp 
from baxter_gazebo.msg import ocg


def mapper(x):
	if x == -1:
		return 'k'
	elif x == 0:
		return 'w'
	else:
		return 'r'

#def unpack(self, grid, width, length, height):
def unpack(data):
	global scale
	global updated
	grid = data.grid
	width = data.width
	length = data.length
	height = data.height
	scale = data.resolution
	global n_x
	global n_y
	global n_z
	global n_c
	global x
	global y
	global z
	global x
	global C
	n_x = []
	n_y = []
	n_z = []
	n_c = []
	for i in range(length):
		for j in range(width):
			for k in range(height):
				ind = i + j*length + k*length*width
				if (grid[ind] != -1 and grid[ind] != 0):
					n_c.append(mapper(grid[ind]))
					n_x.append(i*scale)
					n_y.append(j*scale)
					n_z.append(k*scale)
	print("unpacked...")
	print("Length of x: %d"%len(n_x))
	print("Length of length*width*height: %d"%(length*width*height))
	print("Printing part of n_c: " )
	#print(n_c)
	updated = True
	#return [n_x, n_y, n_z, n_c]			
	x = np.array(n_x).reshape((-1,1))
	y = np.array(n_y).reshape((-1,1))
	z = np.array(n_z).reshape((-1,1))
	#c = np.array(n_c).reshape((-1,1))	
	C = n_c

if __name__ == "__main__":

	rospy.init_node("Plotter")
	rospy.Subscriber("/occupancy_grid", ocg, unpack)	
	
	N = 100 # the initial size of the plotting object
	x = np.array([i for i in range(N)]).reshape((-1,1))
	y = np.array([i for i in range(N)]).reshape((-1,1))
	z = np.array([i for i in range(N)]).reshape((-1,1))
	C = np.array(['k' for i in range(N)]).reshape((-1,1))
	n_x = []
	n_y = []
	n_z = []
	n_c = []
	updated = False
	scale = 0.1
	mul = 10
	
	r = rospy.Rate(5) # frequency of 1 Hz
	fig = plt.figure()
	#ax = Axes3D(fig)  #fig.add_subplot(111, projection='3d')
	plt.cla()
	ax = fig.add_subplot(111, projection='3d')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.ion()
	plt.show()
	updated = False
	while not rospy.is_shutdown():		
#		if updated:
#			global n_x
#			global n_y
#			global n_z
#			global n_c
#			x, y, z, c = [n_x, n_y, n_z, n_c]
#			x = np.array(x).reshape((-1,1))
#			y = np.array(y).reshape((-1,1))
#			z = np.array(z).reshape((-1,1))
#			c = np.array(c).reshape((-1,1))
#			updated = False
#			print("Size of x: %d"%len(x))
		if updated:
			#ax.scatter(x, y , z, '.', c=c, s =int( mul*scale ), marker='o')
			ax.scatter(x, y, z, c=C ) #, s=int(mul*scale) )
			#ax.plot3D(x, y, z, '.') #, color=n_c)
			plt.draw()
			updated = False
			print("Drawing...")
			plt.pause(0.00000000001) 
 		r.sleep()
