import numpy as np
#Defines a velocity profile class

#currently just uses Shelley values
class VelocityProfile:
	def __init__(self, profileType):
		self.type = profileType
		self.s = np.array([[0]])
		self.Ux = np.array([[0]]) 
		self.Ax = np.array([[0]])


	def generate(self, vehicle, path):
		if self.type is "racing":
			self.s, self.Ax, self.Ux = generateRacingProfile(Vehicle, Path)

		else: 
			print("Only racing currently supported")



def generateRacingProfile(vehicle, path):
	#Extract Peformance Limits and parameters
	g = Vehicle.g
	vMax = vehicle.maxSpeed
	AxMax = Path.friction * g
	AyMax = Path.friction * g
	K = path.curvature
	s = path.s

	numSteps = Path.s.size
	
	#Pre-allocate three velocity profiles (steady state, braking, decel)
	UxInit1 = np.zeros((numSteps,1))
	UxInit2 = np.zeros((numSteps,1))
	UxInit3 = np.zeros((numSteps,1))

	#Pre-allocate Ax and Ay
	ax = np.zeros((numSteps,1))
	ay = np.zeros((numSteps,1))

	#Desired velocity should meet lateral acceleration requirement
	UxInit1 = np.sqrt ( np.Divide(AyMax, np.Abs(K + 1e-8) ) )

	#Integrate forward to find acceleration limit
	for i in range(UxInit2.size):
		temp = np.sqrt( UxInit2[i]**2 + 2*AxMax*(s[i+1] - s[i]))
		
		if temp > vMax:
			temp = vMax

		if temp > UxInit1[i+1]:
			temp = UxInit1[i+1]

		UxInit2[i+1] = temp

	#Moving rearward, integrate backwards
	for i = Nsteps:-1:2:
		temp = np.sqrt( UxInit3[i]**2 + 2* AxMax * (s[i] - s[i-1]) )
		if temp > UxInit2[i-1]:
			temp = UxInit2[i-1]

		UxInit3[i-1] = temp


		f


















