#! c:/Python27/python.exe
# -*- coding: utf-8 -*- 

#------------import modules

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.ticker import *
from mpl_toolkits.mplot3d import Axes3D

import csv
	
import scipy as sp
import scipy.interpolate
from matplotlib.mlab import griddata
import matplotlib.mlab as ml


import numpy as np
from math import * 
#import pyemf
import math
#import func
import random 


def drange(begin, end, step):

	n = begin
	value = []

	while n+step < end:
		n += step
		value.append(n)

	return value


def cmtoinch(x):
	
	y = x/2.4
	
	return y

if __name__ == "__main__":
	
	#plot graph
	fig = plt.figure(1)
	ax = fig.gca(projection='3d')

	ax.set_xlim([-100, 100])
	ax.set_ylim([-100, 100])
	ax.set_zlim([-100, 100])
	
	point = ([0.0, 0.0, 0.0],	
                [0.0, -330.0, -65.0],
                [-225.0, 170.0, -135.0],
                [225.0, 170.0, -135.0],
                [-150.0, -150.0,-125.0],
                [150.0, -150.0, -125.0])

	facemodelscale = 0.225;
	point2 = ([0.0*facemodelscale, 0.0*facemodelscale, 0.0*facemodelscale],
                [0.0*facemodelscale, -330.0*facemodelscale, -65.0*facemodelscale],
                [-225.0*facemodelscale, 170.0*facemodelscale, -135.0*facemodelscale],
                [225.0*facemodelscale, 170.0*facemodelscale, -135.0*facemodelscale],
                [-150.0*facemodelscale, -150.0*facemodelscale,-125.0*facemodelscale],
                [150.0*facemodelscale, -150.0*facemodelscale, -125.0*facemodelscale])

	"""
	ax.plot(point[0][0:1], point[0][1:2], point[0][2:3], "o", color="g", ms=4, mew=0.5)
	ax.plot(point[1][0:1], point[1][1:2], point[1][2:3], "o", color="g", ms=4, mew=0.5)
	ax.plot(point[2][0:1], point[2][1:2], point[2][2:3], "o", color="r", ms=4, mew=0.5)
	ax.plot(point[3][0:1], point[3][1:2], point[3][2:3], "o", color="r", ms=4, mew=0.5)
	ax.plot(point[4][0:1], point[4][1:2], point[4][2:3], "o", color="g", ms=4, mew=0.5)
	ax.plot(point[5][0:1], point[5][1:2], point[5][2:3], "o", color="g", ms=4, mew=0.5)
	"""

	ax.plot(point2[0][0:1], point2[0][1:2], point2[0][2:3], "o", color="b", ms=4, mew=0.5)
	ax.plot(point2[1][0:1], point2[1][1:2], point2[1][2:3], "o", color="b", ms=4, mew=0.5)
	ax.plot(point2[2][0:1], point2[2][1:2], point2[2][2:3], "o", color="k", ms=4, mew=0.5)
	ax.plot(point2[3][0:1], point2[3][1:2], point2[3][2:3], "o", color="k", ms=4, mew=0.5)
	ax.plot(point2[4][0:1], point2[4][1:2], point2[4][2:3], "o", color="b", ms=4, mew=0.5)
	ax.plot(point2[5][0:1], point2[5][1:2], point2[5][2:3], "o", color="b", ms=4, mew=0.5)


	plt.show()