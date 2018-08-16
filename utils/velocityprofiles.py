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

		if self.type is "racingSubosits":
			self.s, self.Ax, self.ux = generateRacingProfileSubosits(vehicle, path, friction, vMax)


		else:
			print("Error")



#Uses Subosits and Mick's racing controller accounting for bank and grade, brake factor, weight transfer. 
def generateRacingProfileSubosits(vehicle, path, friction, vMax, jerkLimit = 800, ds = 0.5):
	#Jerk limit in 800 m/s^3, how fast car an come off the brakes
	topoData = np.zeros((2,7))
	topData[1,0] = path.s[-1] #just create dummy data - no need to simulate bank and grade here


	s, Fv2, G, Mv2, Mvdot, theta = makePath3D(path, topodata, vehicle, ds)



# Combines the two-dimensional path data with the topography data
#to make the path 3D.
#   mapData is the map read in according to InitMap.m, topoData is topograghy
#   data as used in InitSpeedProfile.m, and ds is the desired spacing
#   between computation points.  s is a vector of distances along the path
#   that indexes the other outputs.  

def makePath3D(path, topodata, vehicle, ds):
	L = path.s[-1]
	numPoints = floor(L / ds)
	s = np.linspace(0, L, numPoints)

	#grade info wrt s
	theta = np.interp(s, topoData[:,0], topoData[:,1])
	dtheta_ds = np.interp(s, topoData[:,0], topoData[:,2])
	d2theta_ds = np.interp(s, topoData[:,0], topoData[:,3]) 

	#bank info wrt s
	phi = np.interp(s, topoData[:,0], topoData[:,4])
	dphi_ds = np.interp(s, topoData[:,0], topoData[:,5])
	d2phi_ds2 = np.interp(s, topoData[:,0], topoData[:,6])

	#heading info wrt s
	psi = np.interp(s, path.s, path.roadPsi)
	dpsi_ds = np.interp(s, path.s, path.K)
	dpsi_ds2 = np.divide( np.diff(dpsi_ds) , np.diff(s) )
	dpsi_ds2 = np.insert(dpsi_ds2, 0, 0) #prepend zero to keep same size

	#find coefficients for the forces
	Fv2, G = _calcderivatives(psi, theta, phi, dpsi_ds, dtheta_ds)

	#find coefficients for the moments
	Ib = np.diag( np.array([0.1 * veh.*Iz 0.8*veh.Iz veh.Iz]) )
	[Mv2, Mvdot] = _rotaryderivatives(Ib, theta, phi, dpsi_ds, dtheta_ds, dphi_ds, d2psi_ds2, d2theta_ds2, d2phi_ds2)

	return s, Fv2, G, Mv2, Mvdot, theta




    # %CalcDerivatives returns vectors for acclerations in the body frame
    # %   Fv2 is the pointwise acceleration from curvature. Fvdot is the
    # %   acceleration from gas/brake and is [1;0;0] in every column.  G is the
    # %   direction of gravity in the body frame (positive down).  


def _calcderivatives(psi, theta, phi, dpsi, dtheta):
	N = len(psi)

	#path derivatives
	dx = np.dot (np.cos(theta) , np.cos(psi) )
	dy = np.dot (np.cos(theta) , np.sin(psi) )
	dz = -np.sin(theta)

    dx2_ds2 = - np.multiply( np.sin(theta), np.cos(theta), np.cos(psi), np.dtheta_dsigma)  - np.multiply(np.cos(theta), np.cos(theta) , np.sin(psi), dpsi)
    dy2_ds2 =   np.multiply( np.cos(theta), np.cos(theta), np.cos(psi), dpsi_dsigma     )  - np.multiply(np.sin(theta), np.cos(theta) , np.sin(psi), dtheta);
    dz2_ds2 = - np.multiply(np.cos(theta), np.cos(theta), dtheta_dsigma)

    #initialize output variables
    Fv2 = np.zeros(3, N)
    G = zeros(3, N)

    #define transformation matrix M_BI

    for i in range(n):
    	M1 = np.array([[ np.cos(psi[i]),   np.sin(psi[i]), 0]  , [-np.sin(psi[i]), np.cos(psi[i]), 0 ],  [0, 0, 1]])
    	M2 = np.array  (  [[np.cos(theta[i]) , 0 , -sin(theta[i])]  , [0, 1, 0], [np.sin(theta[i]), 0, np.cos(theta[i]) ]] ) 
    	MB2 = np.array (  [[1, 0, 0]  , [0, np.cos(phi[i]), np.sin(phi[i]) ],[ 0, -sin(phi[i]), cos(phi[i])] ])
    	
    	MBI  = np.matmul ( np.matmul ( MB2, M2) , M1)

    	Fv2[:,i] = np.dot(MBI , np.array([[dx2[i]],[dy2[i]],[dz2[i]]]) 
    	G[:,i]   - np.dot(MBI , np.array([0],[0],[9.81]))

    	return Fv2, G











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






















