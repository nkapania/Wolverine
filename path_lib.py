#This library deals with generating words

import numpy as np
from numpy import genfromtxt
#from scipy.io import loadmat
import tables

class Path:
	def __init__(self): 
		self.roadIC = np.zeros((3,1))  #Heading(rad), Road E(m), Road N(m)
		self.posE = np.array([[0]])  #east coordinates
		self.posN = np.array([[0]])  #north coordinates
		self.roadPsi = np.array ([[0]]) #heading
		self.curvature = np.array ([[0]]) #curvature
		self.s = np.array ([[0]])
		self.type = "open" #does not form a loop
		self.referencePoint =np.zeros((3,1)) #GPS reference point
		self.refPointName = "none" #name of reference point
		self.friction = 1.0 #default value

	#loads map from mat file
	def loadFromCSV(self, pathName):
		x = genfromtxt(pathName, delimiter =",")
		self.s = x[:,0]
		self.curvature = x[:,1]
		self.posE = x[:,2]
		self.posN = x[:,3]
		self.roadPsi = x[:,4]
		self.roadIC = x[0:3,5]


	def setFriction(self, value):
		self.friction = value
		



		






		
