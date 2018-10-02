#! c:/Python27/python.exe
# -*- coding: utf-8 -*- 

#------------import modules

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.ticker import *
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
	
	print ""
	i=0
	j=0
	k=0
	labelnum=0
	cnt0 = []
	t0 = []
	
	cnt1 = []
	t1 = []
	
	thin = 1

	#----Read data
	score_face_label = []
	print "reading score_face_label and t0"
	ScoreFaceLabel = "datatimeuse/scorefacelabel.csv"
	scorefacelabel = csv.reader(open(ScoreFaceLabel,"rb"),delimiter=',')
	for row in scorefacelabel:
		if i>0:
			t0.append(float(row[1]))
		i+=1
	print "data_length with csv header:"+str(i)
	print "time_length:"+str(len(t0))
	#print t0
	i-=1
	scorefacelabel = csv.reader(open(ScoreFaceLabel,"rb"),delimiter=',')
	for row in scorefacelabel:
		if j==i:
			labelnum = len(row)
			while k<labelnum:
				score_face_label.append(row[k])
				k+=1
			#print row
		j+=1
	i=0
	score_face_label.pop(0)
	score_face_label.pop(0)
	print score_face_label
	print "len(score_face_label):"+str(len(score_face_label))
	i=0
	j=0
	k=0
	print "read score_face_label and t0"
	print ""

	score_object_label = []
	print "reading score_object_label"
	ScoreObjectLabel = "datatimeuse/scoreobjectlabel.csv"
	scoreobjectlabel = csv.reader(open(ScoreObjectLabel,"rb"),delimiter=',')
	for row in scoreobjectlabel:
		i+=1
	print "data_length with csv header:"+str(i)
	print "time_length:"+str(len(t0))
	#print t0
	i-=1
	scoreobjectlabel = csv.reader(open(ScoreObjectLabel,"rb"),delimiter=',')
	for row in scoreobjectlabel:
		if j==i:
			labelnum = len(row)
			while k<labelnum:
				score_object_label.append(row[k])
				k+=1
			#print row
		j+=1
	i=0
	score_object_label.pop(0)
	score_object_label.pop(0)
	print score_object_label
	print "len(score_object_label):"+str(len(score_object_label))
	i=0
	j=0
	k=0
	print "read score_object_label"
	print ""

	
	print "reading activity_face_data"
	ActivityFaceData = "datatimeuse/activityscorefacedata.csv"
	activityfacedata = csv.reader(open(ActivityFaceData,"rb"),delimiter=',')
	activity_face_data = []
	i=0
	for row in activityfacedata:	
		if i>0:
			activity_face_data.append(int(row[2]))
		i+=1
	#print activity_face_data
	print "read activity_face_data"
	print ""

	print "reading activity_object_data"
	ActivityObjectData = "datatimeuse/activityscoreobjectdata.csv"
	activityobjectdata = csv.reader(open(ActivityObjectData,"rb"),delimiter=',')
	activity_object_data = []
	i=0
	for row in activityobjectdata:	
		if i>0:
			activity_object_data.append(int(row[2]))
		i+=1
	#print activity_object_data
	print "read activity_object_data"
	i=0
	j=0
	print ""

	"""
	i=0
	remove_index = []
	while i<len(score_face_label):
		if score_face_label[i] == "person":
			remove_index.append(i)
		elif score_face_label[i] == "dining table":
			remove_index.append(i)
		i+=1
	i=0
	
	while i<len(activity_object_data):
		
		if 	activity_face_data[i] = 
			activity_object_data[j] -= 1

		
		i+=1	
	
	print score_face_label
	print remove_index
	
	score_face_label.remove("person")
	score_face_label.remove("dining table")
	"""

	print "ploting activity data"
	size_x = 15 
	size_y = 11
	plt.figure(1,figsize=(cmtoinch(size_x),cmtoinch(size_y)))
	
	#plt.minorticks_on()
	params = {
		'font.family'    :'Times New Roman',
		'axes.labelsize' :18,
		#'title.size' :20,
		'xtick.labelsize':18,
		'ytick.labelsize':18,
		'xtick.major.size':5,
		'xtick.minor.size':0,
		'ytick.major.size':5,
		'ytick.minor.size':0,
		'xtick.major.pad':10,
		'xtick.major.pad':10
	}
	plt.rcParams.update(params)		

	plt.xlabel("time[s]")
	plt.ylabel("class")
	i=0
	j=0
	#plt.plot(t0,activity_face_data, label="face")
	#plt.plot(t0,activity_object_data, label="object")
	#plt.scatter(t0, activity_face_data, label="face",marker=".",c=(random.random(),random.random(),random.random()))	
	#plt.scatter(t0, activity_object_data, label="object",marker=".",c=(random.random(),random.random(),random.random()))	
	plt.scatter(t0, activity_face_data, label="face",marker=".",c=(1.0,0,0))	
	plt.scatter(t0, activity_object_data, label="object",marker=".",c=(0,0,1.0))	
	

	plt.subplots_adjust(left=0.15, bottom=0.16,right=0.96, top=0.95)
	i=0
	while i<len(score_face_label):
		#if (score_face_label[i]!="person") and (score_face_label[i]!="dining table"):
		plt.text(3, -1-0.5*i, "class"+str(i)+":"+score_face_label[i])
		i+=1

	plt.axis([0,max(t0),math.floor(-1-0.5*i),len(score_face_label)])
	ymin=(math.floor(-1-0.5*i))
	ymax=len(score_face_label)

	plt.yticks(np.arange(ymin, ymax, 1))
	plt.legend(loc="lower right")
	plt.savefig("activitydata"+".png")
