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
import sys

import numpy as np
from math import * 
#import pyemf
import math
#import func 
import random

#------------CSV open

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

	argv = sys.argv
	argc = len(argv)
	if (argc != 3):
		print 'Usage: python %s arg1 arg2' %argv[0]
		quit()
	data_label = argv[1]
	if argv[2]=="cellphone":
		detect_lable = "cell phone"
	else:
 		detect_lable = argv[2]

	print data_label
	print detect_lable
	
	#----Read data
	score_label = []
	print "reading score_"+data_label+"_label and t0"
	ScoreLabel = "datatimeuse/score"+data_label+"label.csv"
	scorelabel = csv.reader(open(ScoreLabel,"rb"),delimiter=',')
	for row in scorelabel:
		if i>0:
			t0.append(float(row[1]))
		i+=1
	print "data_length with csv header:"+str(i)
	print "time_length:"+str(len(t0))
	#print t0
	i-=1
	scorelabel = csv.reader(open(ScoreLabel,"rb"),delimiter=',')
	for row in scorelabel:
		if j==i:
			labelnum = len(row)
			while k<labelnum:
				score_label.append(row[k])
				k+=1
			#print row
		j+=1
	i=0
	score_label.pop(0)
	score_label.pop(0)
	print score_label
	print "len(score_label):"+str(len(score_label))
	i=0
	j=0
	k=0
	print "read score_"+data_label+"_label and t0"
	print ""
	
	print "reading score_"+data_label+"_data"
	ScoreData = "datatimeuse/score"+data_label+"data.csv"
	scoredata = csv.reader(open(ScoreData,"rb"),delimiter=',')
	for row in scoredata:	
		i+=1
	print "data_length:"+str(i)
	i-=1
	scoredata = csv.reader(open(ScoreData,"rb"),delimiter=',')
	for row in scoredata:
		if j==i:
			labelnum = len(row)
			#print row
		j+=1
	print "labelnum:"+str(labelnum)
	score_data = []
	notemptycnt = 0
	data_length = []
	i=2
	j=0
	k=0
	while i<labelnum:
		j=0
		k=0
		array_data_tmp = []
		notemptycnt=0
		scoredata = csv.reader(open(ScoreData,"rb"),delimiter=',')
		for row in scoredata:
			if j > 0:
				if i<(labelnum-(labelnum-len(row))):
					if row[0] != '':
						array_data_tmp.append(row[i])
					else:
						array_data_tmp.append('0')
					notemptycnt+=1
				else:
					array_data_tmp.append(0)
			j+=1
		#print "i="+str(i)+" "+"j="+str(j)+" len(array_data_tmp)before:"+str(len(array_data_tmp))
		#print array_data_tmp
		while k<j-1:
			array_data_tmp[k] = int(array_data_tmp[k])
			k+=1
		data_length.append(k)
		#print "i="+str(i)+" array_data_tmp_after"
		#print array_data_tmp
		score_data.append(array_data_tmp)
		i+=1
	#print scoredata
	i=0
	j=0
	k=0
	alldatalength = 0
	for x in score_data:
		alldatalength += len(x)
	print "alldatalength:"+str(alldatalength)
	#print score_data[0][:]
	print "read score_"+data_label+"_data"
	
	print ""

	print "ploting score_"+data_label+"_data"
	size_x = 40
	size_y = 11
	r=[]
	g=[]
	b=[]
	figurenum  = 1
	plt.figure(figurenum,figsize=(cmtoinch(size_x),cmtoinch(size_y)))
	
	#plt.minorticks_on()
	params = {
		'font.family'    :'Bitstream Vera Sans',
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
	plt.ylabel("score")

	x_data = []
	y_data = []
	i=0
	j=0
	while i<len(t0):
		x_data.append(t0[i])
		i+=1

	i=0
	plt.axis([0,max(x_data),-1.2,1.2])	
	while i<len(score_label):
		y_data = []
		j=0
		if (score_label[i]!="person") and (score_label[i]!="dining table"):
			while j<len(t0):
				y_data.append(score_data[i][j])
				j+=1
			r.append(random.random())
			g.append(random.random())
			b.append(random.random())
			plt.scatter(x_data,y_data, label=score_label[i],marker=".")	
		i+=1
	
	plt.subplots_adjust(left=0.15, bottom=0.16,right=0.96, top=0.95)
	plt.legend(loc="lower right")
	plt.savefig(data_label+"score/"+data_label+"scoredata"+".png")
	figurenum +=1
	
	i=0
	j=0
	k=0
	print "ploted score_"+data_label+"_data"
	print ""
	
	print "ploting each score_"+data_label+"_data"
	y_data = []
	i=0
	j=0
	while i<len(score_label):
		y_data = []
		j=0
		if (score_label[i]!="person") and (score_label[i]!="dining table"):
			while j<len(t0):
				y_data.append(score_data[i][j])
				j+=1
			plt.figure(figurenum,figsize=(cmtoinch(size_x),cmtoinch(size_y)))
			params = {
			'font.family'    :'Bitstream Vera Sans',
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
			plt.ylabel("score")
			plt.axis([0,max(x_data),-1.2,1.2])	
			plt.scatter(x_data,y_data,marker=".")
			plt.subplots_adjust(left=0.15, bottom=0.16,right=0.96, top=0.95)
			plt.savefig(data_label+"score/"+data_label+"_"+score_label[i]+".png")
	
		i+=1
		figurenum+=1
	print "ploted each score_"+data_label+"_data"
	print ""

	print "measurement result"
	print data_label
	print detect_lable
	detected_cnt = 0
	i=0
	while i<len(score_label):
		j=0
		if score_label[i] == detect_lable:
			while j<len(t0):
				if score_data[i][j] > 0:
					detected_cnt+=1
				j+=1
		i+=1
	
	print "all data num:"+str(len(t0))
	print "detected num:"+str(detected_cnt)
	print "rate:"+str((float(detected_cnt)/float(len(t0)))*100)
	