import numpy as np
import matplotlib.pyplot as plt
#Defines a velocity profile class

#currently just uses Shelley values
class VelocityProfile:
	def __init__(self, profileType):
		self.type = profileType
		self.s = np.array([[0]])
		self.Ux = np.array([[0]]) 
		self.Ax = np.array([[0]])


	def generate(self, vehicle, path, friction = 0.3, vMax = 10):
		if self.type is "racing":
			if path.isOpen:
				self.s, self.Ax, self.Ux = generateRacingProfileOpen(vehicle, path, friction, vMax)
			else:
				self.s, self.Ax, self.Ux = generateRacingProfileClosed(vehicle, path, friction, vMax)

		else:
			print("Error")




def generateRacingProfileOpen(vehicle, path, friction, vMax):
	Ux, Ax = _genSpeedHelper(path.s, path.curvature, friction, 0, vMax) #minimum velocity is zero
	return path.s, Ax, Ux


def generateRacingProfileClosed(vehicle, path, friction, vMax):
	g = 9.81
	K = path.curvature
	s = path.s
	AyMax = friction * g


	#calculate lowest velocity point
	UxSS = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )
	minUx = np.amin(UxSS)
	idx = np.argmin(UxSS)

	#shift so we generate starting at lowest point
	inds = np.arange(len(UxSS))
	shiftedInds = np.roll(inds, -idx)
	kShifted = K[shiftedInds]

	UxShift, AxShift = _genSpeedHelper(s, kShifted, friction, minUx, vMax)

	#unshift back to original
	Ux = np.roll(UxShift, idx)
	Ax = np.roll(AxShift, idx)

	return s, Ax, Ux


def _genSpeedHelper(s, K, friction, minUx, maxUx):
	#Extract Peformance Limits and parameters
	g = 9.81
	AxMax = friction * g
	AyMax = friction * g
	
	numSteps = s.size
	
	#Pre-allocate three velocity profiles (steady state, braking, decel)
	UxInit1 = np.zeros(numSteps)
	UxInit2 = np.zeros(numSteps); UxInit2[0]  = minUx
	UxInit3 = np.zeros(numSteps); UxInit3[-1] = minUx 

	#Pre-allocate Ax and Ay
	ax = np.zeros(numSteps)
	ay = np.zeros(numSteps)


	#Desired velocity should meet lateral acceleration requirement
	UxInit1 = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )

	# #Integrate forward to find acceleration limit
	for i in range(UxInit2.size-1):
	 	temp = np.sqrt( UxInit2[i]**2 + 2*AxMax*(s[i+1] - s[i]))
		
	 	if temp > maxUx:
	 		temp = maxUx

	 	if temp > UxInit1[i+1]:
	 		temp = UxInit1[i+1]

	 	UxInit2[i+1] = temp

	#Moving rearward, integrate backwards
	for i in reversed(range(1,UxInit3.size)):
		temp = np.sqrt( UxInit3[i]**2 + 2* AxMax * (s[i] - s[i-1]) )
		

		if temp > UxInit2[i-1]:
			temp = UxInit2[i-1]

		UxInit3[i-1] = temp



	#calculate acceleration profile from physics
	ax = np.divide( (np.roll(UxInit3,1)**2 - UxInit3**2) , (2 * (np.roll(s,1) - s) ) )
	ax[0] = ax[1] #avoid bug where vehicle starts with initial desired acceleration
	

	return UxInit3, ax






















