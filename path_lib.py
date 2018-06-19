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
		s = x[:,0]
		curvature = x[:,1]
		posE = x[:,2]
		posN = x[:,3]
		roadPsi = x[:,4]
		roadIC = x[0:3,5]

		#reshape to row vectors
		self.s = np.reshape(s, (s.size,1) )
		self.curvature = np.reshape(curvature, (curvature.size,1) )
		self.posE = np.reshape(posE, (posE.size,1) )
		self.posN = np.reshape(posN, (posN.size,1) )
		self.roadPsi = np.reshape(roadPsi, (roadPsi.size,1) )
		self.roadIC = np.reshape(roadIC, (roadIC.size,1) )






	def setFriction(self, value):
		self.friction = value
		



		






		
