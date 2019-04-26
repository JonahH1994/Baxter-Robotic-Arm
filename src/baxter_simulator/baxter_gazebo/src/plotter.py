#!/usr/bin/env python
# coding=utf8
import matplotlib
matplotlib.use('Agg')
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import rospy
from baxter_gazebo.srv import OccupancyGridPlot as ogp 

class Plotter:
	
	def __init__(self):
		#rospy.init_node('Plotter')
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(111, projection='3d')
		self.mul = 10 # so that 10 * scale = the size of the marker
		self.ax.set_xlabel('X')
		self.ax.set_ylabel('Y')
		self.ax.set_zlabel('Z')
		rospy.Service('Plotter', ogp, self.parser)	

	def mapper(self, x):
		# maps -1 to black and 1 to red
		if x == -1:
			return 'k'
		else:
			return 'r'

	def unpack(self, grid, width, length, height):
	#def unpack(self, grid, pmin, scale, width, length, height):	
		x = []
		y = []
		z = []
		c = []
		for i in range(length):
			for j in range(width):
				for k in range(height):
					x.append(i)
					y.append(j)
					z.append(k)
					ind = i + j*length + k*length*width
					#ind = i + j + k
					c.append(self.mapper(grid[ind]))
		return [x, y, z, c]			

	def parser(self, data):
		grid = data.grid
		width = data.width
		length = data.length
		height = data.height
		scale = data.scale

		x,y,z,c = self.unpack(grid, width, length, height)
		self.ax.scatter(x, y, z, c=c, s=int(self.mul*scale), marker='o')
		plt.draw()
		plt.pause(0.000000001)
		#plt.show()

if __name__ == '__main__':
	rospy.init_node('Plotter')
	plotter = Plotter() 
	plt.ion()
	plt.show()
	rospy.spin()
