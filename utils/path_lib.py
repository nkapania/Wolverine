#This library deals with generating words

import numpy as np
from numpy import genfromtxt
import scipy.io as sio


class Path:
	def __init__(self): 
		self.roadIC = np.zeros((3,1))  #Heading(rad), Road E(m), Road N(m)
		self.posE = [0]  #east coordinates
		self.posN = [0]  #north coordinates
		self.roadPsi = [0] #heading
		self.curvature = [0] #curvature
		self.s = [0]
		self.type = "open" #does not form a loop
		self.referencePoint =np.zeros((3,1)) #GPS reference point
		self.refPointName = "none" #name of reference point
		self.friction = 1.0 #default value

	#loads map from csv file - CRUDE
	def loadFromCSV(self, pathName):
		x = genfromtxt(pathName, delimiter =",")
		self.s = np.array(x[:,0])
		self.curvature = np.array(x[:,1])
		self.posE = np.array(x[:,2])
		self.posN = np.array(x[:,3])
		self.roadPsi = np.array(x[:,4])
		self.roadIC = np.array(x[0:3,5])

	def loadFromMAT(self, pathName):
		path = sio.loadmat(pathName, squeeze_me = True)
		self.s = path['world']['s'].sum()
		self.curvature = path['world']['K'].sum()
		self.posE = path['world']['roadE'].sum()
		self.posN = path['world']['roadN'].sum()
		self.roadPsi = path['world']['roadPsi'].sum()
		self.roadIC = path['world']['road_IC'].sum()




	def setFriction(self, value):
		self.friction = value
		



		






		
