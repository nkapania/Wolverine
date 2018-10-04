#This library deals with generating words

import numpy as np
from numpy import genfromtxt
import scipy.io as sio
from scipy.integrate import odeint


class Path:
	def __init__(self): 
		self.roadIC = np.zeros((3,1))  #Heading(rad), Road E(m), Road N(m)
		self.posE = [0]  #east coordinates
		self.posN = [0]  #north coordinates
		self.roadPsi = [0] #heading
		self.curvature = [0] #curvature
		self.s = [0]
		self.isOpen = True #open = whether track forms loop or not
		self.roadIC =np.zeros((3,1)) #GPS reference point
		self.refPointName = None #name of reference point
		self.friction = None  #initialize to none
		self.vMax = None

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
		self.s = path['world']['s'].sum() #No idea why you need the .sum, but otherwise doesn't work
		self.curvature = path['world']['K'].sum()
		self.posE = path['world']['roadE'].sum()
		self.posN = path['world']['roadN'].sum()
		self.roadPsi = path['world']['roadPsi'].sum()
		self.roadIC = path['world']['road_IC'].sum()
		self.isOpen = bool(path['world']['isOpen'].sum())
		
		#Optional fields: Max Velocity
		try:
			self.vMax = path['world']['vMax'].sum()
		except:
			self.vMax = None
		
		#Friction
		try:
			self.friction = path['world']['friction'].sum()
		except:
			self.friction = None

		#Bank and grade
		try:
			self.bank = path['world']['bank'].sum()
			print('Loaded Bank Information')
		except:
			self.bank = np.zeros((self.s.size, 3))

		try:
			self.grade = path['world']['grade'].sum()
			print('Loaded Grade Information')
		except:
			self.grade = np.zeros((self.s.size, 3))



	def genFromSK(self, prim_s, prim_k, points_per_meter = 4):
		
		#resample
		n = points_per_meter * prim_s[-1]
		s = np.linspace(prim_s[0], prim_s[-1],n)
		k = np.interp(s, prim_s, prim_k)

		# initial condition
		E_init   = 0
		N_init   = 0
		psi_init = 0

		# create path
		psi, E, N = integratePath(s, k, E_init, N_init, psi_init)

		self.s = s
		self.curvature = k
		self.posE = E
		self.posN = N
		self.roadPsi = psi
		self.roadIC = [psi_init, E_init, N_init]
		self.isOpen = 1 #worlds generated from SK are always open


		return None

	def generateRandomWorld(self, numTurns = 5, ds = 0.5, k_min = 0, k_max = .1, s_min = 50, s_max = 200):
		s, k = generateRandomClothoid(ds, k_min, k_max, s_min, s_max)

		for i in range(1,numTurns):
			si, ki = generateRandomClothoid(ds, k_min, k_max, s_min, s_max)
			si = si + s[-1] + ds

			s = np.concatenate((s, si))
			k = np.concatenate((k, ki))


		self.genFromSK(s, k)

	def resample(self, ds):
		#resample path to a different constant spacing

		s = np.arange(0, self.s[-1], ds)
		curvature = np.interp(s, self.s, self.curvature)
		posE = np.interp(s, self.s, self.posE)
		posN = np.interp(s, self.s, self.posN)
		roadPsi = np.interp(s, self.s, self.roadPsi)
		
		#not sure the best way to do these next six lines
		b1 = np.interp(s, self.s, self.bank[:,0])
		b2 = np.interp(s, self.s, self.bank[:,1])
		b3 = np.interp(s, self.s, self.bank[:,2])
		g1 = np.interp(s, self.s, self.grade[:,0])
		g2 = np.interp(s, self.s, self.grade[:,1])
		g3 = np.interp(s, self.s, self.grade[:,2])

		bank = np.concatenate((b1[:, np.newaxis], b2[:,np.newaxis], b3[:,np.newaxis]), axis = 1)
		grade = np.concatenate((g1[:, np.newaxis], g2[:, np.newaxis], g3[:, np.newaxis]), axis = 1)

		self.s = s
		self.curvature = curvature
		self.posE = posE
		self.posN = posN
		self.roadPsi = roadPsi
		self.bank = bank
		self.grade = grade

		return




###################################################################################################
################################### HELPER FUNCTIONS ##############################################
###################################################################################################


def generateRandomClothoid(ds, k_min, k_max, s_min, s_max):
	l_straight = sampleLength(s_min, s_max)
	l_entry    = sampleLength(s_min, s_max)
	l_const_radius = sampleLength(s_min, s_max)
	l_exit = sampleLength(s_min, s_max)
	curvature = sampleCurvature(k_min, k_max)

	s1 = np.arange(0, l_straight, ds)
	s2 = s1[-1] + np.arange(ds, l_entry, ds)
	s3 = s2[-1] + np.arange(ds, l_const_radius, ds)
	s4 = s3[-1] + np.arange(ds, l_exit, ds)

	k1 = np.zeros(s1.shape)
	k2 = np.linspace(0, curvature, s2.size)
	k3 = np.linspace(curvature, curvature, s3.size)
	k4 = np.linspace(curvature, 0, s4.size)

	s = np.concatenate((s1, s2, s3, s4))
	k = np.concatenate((k1, k2, k3, k4))

	return s, k

def sampleCurvature(kmin, kmax):
	K = kmin + np.random.uniform() * kmax
	K = K * np.sign(np.random.randn())
	return K

def sampleLength(Lmin, Lmax):
	L = Lmin + np.random.uniform()*Lmax
	return L

def pathDerivs(y, s, S, K):
    # y = [psi, E, N]
    k = np.interp(s, S, K)
    dyds = [k, -np.sin(y[0]), np.cos(y[0])]
    return dyds

def integratePath(s, k, E_init, N_init, psi_init):
    y0 = [psi_init, E_init, N_init]
    sol = odeint(pathDerivs, y0, s, args=(s,k))
    psi = sol[:,0]
    E = sol[:,1]
    N = sol[:,2]
    return (psi, E, N)







	














		
