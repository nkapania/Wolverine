import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
from numpy import cos
from numpy import sin
#Defines a velocity profile class

class RecordedProfile():
	def __init__(self, matFileName):
		mfile = sio.loadmat(matFileName)
		self.s =  mfile["s"]
		self.Ux = mfile["Ux"]
		try:
			self.Ax = mfile["Ax"]
		except:
			self.Ax = np.zeros(self.s.shape)


#This speed profile generation algorithm is based on a simple "3 pass" method that accounts for steady state speeds
#and integration according to a friction circle. It is best used for educational purposes or for very simple lanekeeping
#demonstrations well below the limits. It tends to cause instability in braking regions due to improper handling of weight
#transfer and generally aggressive braking in the straight segments. 

class BasicProfile():
    def __init__(self, vehicle, path, friction = 0.3, vMax = 10., AxMax = 9.81):
		self.vehicle = vehicle
		self.path = path

	    #initialize variables
		self.s = path.s
		self.Ux = np.zeros(self.s.shape)
		self.Ax = np.zeros(self.s.shape)

		if isinstance(vMax, np.ndarray):
			self.vMax = vMax
		else: 
			self.vMax = vMax * np.ones(self.s.shape)

		if isinstance(friction, np.ndarray):
			self.mu = friction
		else:
			self.mu = friction * np.ones(self.s.shape)



	    
		if path.isOpen:
			self.generateBasicProfileOpen(AxMax)

		else:
			self.generateBasicProfileClosed(AxMax)

    def generateBasicProfileClosed(self, AxMax):
	    g = 9.81
	    K = self.path.curvature
	    s = self.s
	    AyMax = self.mu* g
	    AxMax = min( np.append(self.mu * g, abs(AxMax)))

	    #calculate lowest velocity point
	    UxSS = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )
	    minUx = np.amin(UxSS)
	    maxUx = self.vMax
	    idx = np.argmin(UxSS)

	    #shift so we generate starting at lowest point
	    inds = np.arange(len(UxSS))
	    shiftedInds = np.roll(inds, -idx)
	    kShifted = K[shiftedInds]
	    maxUxShifted = maxUx[shiftedInds]
	    AyMaxShifted = AyMax[shiftedInds]

	    UxShift, AxShift = self.genSpeed(kShifted, minUx, maxUxShifted, AxMax, AyMaxShifted)

	    #unshift back to original
	    self.Ux = np.roll(UxShift, idx)
	    self.Ax = np.roll(AxShift, idx)

	    return

    def generateBasicProfileOpen(self, AxMax):
    	g = 9.81
    	K = self.path.curvature
    	AyMax = self.mu* g
    	AxMax = min( np.append(self.mu * g, abs(AxMax)))
    	self.Ux, self.Ax = self.genSpeed(K, 0, self.vMax, AxMax, AyMax) #minimum velocity is zero

    def genSpeed(self, K, minUx, maxUx, AxMax, AyMax):
	    #Extract Peformance Limits and parameters
	    g = 9.81
	    s = self.s
	    
	    numSteps = s.size
	    
	    #Pre-allocate three velocity profiles (steady state, braking, decel)
	    UxInit1 = np.zeros(numSteps)
	    UxInit2 = np.zeros(numSteps) 
	    UxInit2[0]  = minUx
	    UxInit3 = np.zeros(numSteps) 
	    UxInit3[-1] = minUx 

	    #Pre-allocate Ax and Ay
	    ax = np.zeros(numSteps)
	    ay = np.zeros(numSteps)


	    #Desired velocity should meet lateral acceleration requirement
	    UxInit1 = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )

	    # #Integrate forward to find acceleration limit
	    for i in range(UxInit2.size-1):
	         temp = np.sqrt( UxInit2[i]**2 + 2*AxMax*(s[i+1] - s[i]))
	         
	         if temp > maxUx[i]:
	             temp = maxUx[i]

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



#This is a more sophisticated velocity profile generation algorithm that accounts for weight 
#transfer, front and rear wheel friction limitations, and bank and grade

#Attribution: John Subosits, Mick Kritayakirana

class RacingProfile():
    def __init__(self, vehicle, path, friction = 0.3, vMax = 10):
        self.vehicle = vehicle
        self.path = path
        self.mu = friction
        self.vMax = vMax

        #initialize variables
        self.s = path.s
        self.Ux = np.zeros(self.s.shape)
        self.Ax = np.zeros(self.s.shape)

        #self.getRacingProfile()


    def getRacingProfile(self):

    	#compute array of ds values

    	Fv2, G, Mv2, MvDot, theta = self.makePath3D()
    	self.findSpeedProfile(Fv2, G, Mv2, Mvdot, theta)

    def findSpeedProfile(self,Fv2, G, Mv2, Mvdot, theta):
    	
    	#define variables from objects for code readability
    	n = len(self.path.s) #number of sample points
    	Vmax = self.vMax * np.ones(self.s.shape)
    	mu = self.mu * np.ones(self.s.shape)
    	AxDesired = self.Ax
    	s = self.s


	    # handle the open maps by limiting the final speed to zero for last 10 m
	    if self.path.isOpen
	        stoppedPoints = s >= (s[-1] - self.stopDistance)
	        Vmax[stoppedPoints] = 0
	    
	    else # remove last point to prevent double counting/duplication
	        # this is probably the cleanest way to do it to avoid extra logic later
	        endPoint = s[-1]
	        
	        s = s[:-1]
	        Fv2 = Fv2[:,:-1]
	        G = G[:,:-1]
	        Mvdot = Mvdot[:,:-1]
	        Mv2 = Mv2[:,:-1]
	        theta = theta[:-1]
	        AxDesired = AxDesired[:-1]
	        Vmax      = Vmax[:-1]
	        mu        = mu  [:-1]
	    

	    # find velocity max based on pointwise friction constraint
	    algebraicVmax = calcVmax(0, self.vehicle, mu, Fv2, G, Mv2[1:,:],Mvdot[1:,:])
	    UxDesired = min(Vmax, algebraicVmax)    #target speed
	    
	    s,UxDesired,AxDesired,AxMax = self.generateSpeedProfile(UxDesired,AxDesired,veh,mu,s,Fv2,G,Mv2,Mvdot,theta)

	    #close the loop and ensure continuity
	    if not path.isOpen
	        s = np.concatenate(s, endPoint)
	        UxDesired = np.concatenate(UxDesired, UxDesired[-1])
	        AxDesired = np.concatenate(AxDesired, AxDesired[-1])
	        AxMax     = np.concatenate(AxMax, AxMax[-1]) 

	    else #set desired acceleration to the negitive limit so that the car stops and continues braking.
	        AxDesired[stoppedPoints] = -mu[stoppedPoints]*self.vehicle.g


	    self.s = s
	    self.Ux = UxDesired
	    self.Ax = AxDesired
    
    	return

    def calcVmax(self, Vdot, mu, fv2, g, mv2, mvdot):
    	return Vmax, Vdot

    def generateSpeedProfile(self, UxDesired,AxDesired,veh,mu,s,Fv2,G,Mv2,Mvdot,theta):
    	return s, UxDesired, AxDesired, AxMax

    def calcHeadingData(self):
		psi = self.path.roadPsi
		dpsi_dsigma = self.path.curvature
		d2psi_dsigma2 = np.diff(dpsi_dsigma) / np.diff(self.path.s)
		d2psi_dsigma2 = np.insert(d2psi_dsigma2, 0, 0)
 
		return psi, dpsi_dsigma, d2psi_dsigma2


    def makePath3D(self):
    	#parameters
    	Iz = self.vehicle.Iz

    	#grade
    	theta = self.path.grade[:,0]
    	dtheta_dsigma = self.path.grade[:,1]
    	d2theta_dsigma2 = self.path.grade[:,2]

    	#bank
    	phi = self.path.bank[:,0]
    	dphi_dsigma = self.path.bank[:,1]
    	d2phi_dsigma2 = self.path.bank[:,2]

    	#get heading at those points
    	psi, dpsi_dsigma, d2psi_dsigma2 = self.calcHeadingData()

    	#find coefficients for the forces
    	[Fv2, _, G] = self.calcDerivatives(psi, theta, phi, dpsi_dsigma, dtheta_dsigma)

    	#find coefficients for the moments
    	Ib = np.diag([0.1 * Iz, 0.8*Iz, Iz]) 
    	[Mv2, Mvdot] = self.rotaryDerivatives(Ib, theta, phi, dpsi_dsigma, dtheta_dsigma, dphi_dsigma, d2psi_dsigma2, d2theta_dsigma2, d2phi_dsigma2)

    	return Fv2, G, Mv2, Mvdot, theta

    def rotaryDerivatives(self, Ib, theta, phi, dpsi_dsigma, dtheta_dsigma, dphi_dsigma, d2psi_dsigma2, d2theta_dsigma2, d2phi_dsigma2):	
		#RotaryDerivatives Calculates coefficients for V^2 and Vdot in the moment equation
	    n = len(phi)
	    #make dimensions compatible
	    

	    # theta = np.reshape(theta,(1,n))
	    # phi   = np.reshape(phi,(1,n))
	    # dpsi_dsigma   = np.reshape(dpsi_dsigma,(1,n))
	    # dtheta_dsigma = np.reshape(dtheta_dsigma,(1,n))
	    # dphi_dsigma   = np.reshape(dphi_dsigma,(1,n))
	    # d2psi_dsigma2 = np.reshape(d2psi_dsigma2,(1,n))
	    # d2theta_dsigma2 = np.reshape(d2theta_dsigma2,(1,n))
	    # d2phi_dsigma2   = np.reshape(d2phi_dsigma2,(1,n))

	    #path coordinate derivatives
	    dphi_ds = dphi_dsigma*cos(theta)
	    dtheta_ds = dtheta_dsigma*cos(theta)
	    dpsi_ds = dpsi_dsigma*cos(theta)
	    d2phi_ds2 = cos(theta)*(d2phi_dsigma2*cos(theta) - dphi_dsigma*dtheta_dsigma*sin(theta))
	    d2theta_ds2 = cos(theta)*(d2theta_dsigma2*cos(theta) - (dtheta_dsigma**2)*sin(theta))
	    d2psi_ds2 = cos(theta)*(d2psi_dsigma2*cos(theta) - dpsi_dsigma*dtheta_dsigma*sin(theta))

	    V2a = np.zeros((3,n))
	    V2b = np.zeros((3,n))
	    V2c = np.zeros((3,n))
	    Mvdot = np.zeros((3,n))


	    for i in range(n):
	        L_BI = np.array([ [1, 0, -sin(theta[i])],  [0,  cos(phi[i]),  sin(phi[i])*cos(theta[i])], [0, -sin(phi[i]), cos(phi[i])*cos(theta[i])] ])
	        wb   = np.dot(  L_BI ,  np.array([ [dphi_ds[i]], [dtheta_ds[i]] , [dpsi_ds[i]] ]) )
	        wbtilde = np.array([[0, -wb[2], wb[1]], [wb[2], 0, -wb[0]], [-wb[2], wb[0], 0]])

	        V2a[:,i] = np.dot( np.dot(wbtilde, Ib), wb ).squeeze()
	        d2vec = np.array([ [d2phi_ds2[i]] , [d2theta_ds2[i]] , [d2psi_ds2[i]] ])

	        V2c[:,i] = np.dot(Ib, np.dot(L_BI, d2vec)).squeeze()  
	        
	        Mvdot[:,i] = np.dot(Ib,  np.dot(L_BI, np.array([ [dphi_ds[i]],  [dtheta_ds[i]], [dpsi_ds[i]]] ))).squeeze()	        
	        L_BIdot = np.array( [[0, 0, -cos(theta[i])*dtheta_ds[i]] , 
			[0, (-sin(phi[i])*dphi_ds[i]), (cos(phi[i])*cos(theta[i])*dphi_ds[i] - sin(phi[i])*sin(theta[i])*dtheta_ds[i])], 
			[0, (-cos(phi[i])*dphi_ds[i]), (-sin(phi[i])*cos(theta[i])*dphi_ds[i] - cos(phi[i]) * sin(theta[i])*dtheta_ds[i])]])

	        V2b[:,i] = np.dot( Ib,   np.dot( L_BIdot, np.array( [ [dphi_ds[i]], [dtheta_ds[i]],  [dpsi_ds[i] ]] ))).squeeze()
	    
	    Mv2 = V2a + V2b + V2c

	    return Mv2, Mvdot

    def calcDerivatives(self, psi, theta, phi, dpsi_dsigma, dtheta_dsigma):
		# CalcDerivatives returns vectors for accelerations in the body frame
	    # Fv2 is the pointwise acceleration from curvature. Fvdot is the
	    # acceleration from gas/brake and is [100] in every column.  G is the
	    # direction of gravity in the body frame (positive down).  
	    
	    n = len(psi)

	    #path coordinate derivatives
	    dx_ds = np.cos(theta)*np.cos(psi)
	    dy_ds = np.cos(theta)*np.sin(psi)
	    dz_ds =-np.sin(theta)

	    dx2_ds2 = -np.sin(theta)*np.cos(theta)*np.cos(psi)*dtheta_dsigma - np.cos(theta)**2*np.sin(psi)*dpsi_dsigma
	    dy2_ds2 = (np.cos(theta)**2)*np.cos(psi)*dpsi_dsigma - np.sin(theta)*np.cos(theta) *np.sin(psi)*dtheta_dsigma
	    dz2_ds2 = -np.cos(theta)**2*dtheta_dsigma

	    #initialize output variables
	    Fv2 =   np.zeros((3,n))
	    Fvdot = np.zeros((3,n))
	    G     = np.zeros((3,n))
	    
		# transform to body frame using rotation matrix M_BI
	    for i in range(len(psi)):
			M_1I = np.array([ [np.cos(psi[i]), np.sin(psi[i]), 0] , [-np.sin(psi[i]), np.cos(psi[i]), 0], [0, 0, 1] ])
			M_21 = np.array([ [np.cos(theta[i]), 0, -np.sin(theta[i]) ], [0, 1, 0 ], [sin(theta[i]), 0, cos(theta[i])] ])

			M_B2 = np.array([ [1, 0, 0] , [0, cos(phi[i]), sin(phi[i])],  [0, -sin(phi[i]), cos(phi[i]) ] ]) 
      
			M_BI = np.matmul(np.matmul(M_B2, M_21), M_1I)

			Fv2[:,i] =   np.dot( M_BI ,  np.array([ [dx2_ds2[i]], [dy2_ds2[i]] , [dz2_ds2[i]]] ) ).squeeze()
			Fvdot[:,i] = np.dot( M_BI ,  np.array([ [dx_ds[i]], [dy_ds[i]], [dz_ds[i]] ]) ).squeeze()
			G[:,i] =     np.dot( M_BI ,  np.array([ [0] , [0] , [self.vehicle.g] ]) ).squeeze()

	    return Fv2, Fvdot, G
		





	       




































