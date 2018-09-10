import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
from numpy import cos
from numpy import sin
import pdb
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
#demonstrations well below the limits. It ts to cause instability in braking regions due to improper handling of weight
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
	def __init__(self, vehicle, path, friction = 0.3, vMax = 10, jerkLimit = 800, ds = 0.5):
		self.vehicle = vehicle 
		self.path = path
		self.path.resample(ds) #resample path to fixed ds 
		self.mu = friction
		self.vMax = vMax #max velocity
		self.jerkLimit = jerkLimit # m / s^3

		#initialize variables
		self.s = path.s
		self.Ux = np.zeros(self.s.shape)
		self.Ax = np.zeros(self.s.shape)

		self.getRacingProfile()


	def getRacingProfile(self):
		print("Generating Racing Profile")
		Fv2, G, Mv2, MvDot, theta = self.makePath3D()
		self.findSpeedProfile(Fv2, G, Mv2, MvDot, theta)

	def findSpeedProfile(self,Fv2, G, Mv2, Mvdot, theta):
		
		#define variables from objects for code readability
		n = len(self.path.s) #number of sample points
		Vmax = self.vMax * np.ones(self.s.shape)
		mu = self.mu * np.ones(self.s.shape)
		AxDesired = self.Ax
		s = self.s

		# handle the open maps by limiting the final speed to zero for last 10 m
		if self.path.isOpen:
			stoppedPoints = s >= (s[-1] - self.stopDistance)
			Vmax[stoppedPoints] = 0

		# remove last point to prevent double counting/duplication
		else: 
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
		algebraicVmax, _ = self.calcVmax(0, mu, Fv2, G, Mv2[1:,:],Mvdot[1:,:])
		UxDesired = np.minimum(Vmax, algebraicVmax)    #target speed
		self.UxDesiredAlgebraic = UxDesired

		s, UxDesired,AxDesired,AxMax = self.generateSpeedProfile(UxDesired,AxDesired,mu,s,Fv2,G,Mv2,Mvdot,theta)

		#close the loop and ensure continuity
		if not self.path.isOpen:
			s = np.concatenate((s.squeeze(), endPoint[np.newaxis]))
			UxDesired = np.concatenate((UxDesired, UxDesired[-1, np.newaxis]))
			AxDesired = np.concatenate((AxDesired, AxDesired[-1, np.newaxis]))
			AxMax     = np.concatenate((AxMax, AxMax[-1, np.newaxis])) 
		else:
			AxDesired[stoppedPoints] = -mu[stoppedPoints]*self.vehicle.g


		self.s = s
		self.Ux = UxDesired
		self.Ax = AxDesired
	
		return

	def calcVmax(self, Vdot, mu, fv2, g, mv2, mvdot):
		#define parameters
		a = self.vehicle.a
		b = self.vehicle.b
		L = a+b    # wheelbase, [m]
		m = self.vehicle.m
		D = self.vehicle.dragCoeff
		hcg = self.vehicle.h 
		beta = self.vehicle.beta   # ratio of front to rear wheel torques may need to reexamine this to decide if it needs to be different for  braking and driving

		# terms in constraint equation
		g1 = g[0,:]
		g2 = g[1,:]
		g3 = g[2,:]
		fv22 = fv2[1,:]
		fv23 = fv2[2,:]
		mv22 = mv2[0,:]
		mv23 = mv2[1,:]
		mvdot2 = mvdot[0,:]
		mvdot3 = mvdot[1,:]

		#front wheels find terms for V**2 equation
		g =  beta*D/(1 + beta)
		h =  beta*(m*g1 + m*Vdot)/(1+beta)
		c =  (b*m*fv22 + mv23)/L
		d =  (b*m*g2 + mvdot3*Vdot)/L
		e =  (m*b*fv23 - mv22 - hcg*D)/L
		f =  (m*b*(g3) - hcg*(m*(g1 + Vdot)) - mvdot2*Vdot )/L

		A = (g)**2 + (c)**2 - mu**2*(e)**2
		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f))
		C = (h)**2 + (d)**2 - mu**2*(f)**2

		VmaxFront = np.zeros(A.shape)
		VmaxRear  = np.zeros(B.shape)

		# use positive solution to quadratic eqn.
		for i in range(len(B)):
			try:
				VmaxFront[i] = np.sqrt((-B[i] + np.sqrt(B[i]**2 - 4*A[i]*C[i]))/(2*A[i]))
				if np.isnan(VmaxFront[i]):
					VmaxFront[i] = self.vMax #handle negative square roots
			except:
				VmaxFront[i] = self.vMax

		# do rear wheels
		mu = mu* self.vehicle.muR / self.vehicle.muF # encodes the steady state understeer of the car
		g = D/(1+beta)
		h = m*(g1+Vdot)/(1+beta)
		c = (a*m*fv22 - mv23)/L
		d = (a*m*g2 - mvdot3*Vdot)/L
		e = (m*a*fv23 + mv22 + hcg*D)/L
		f = (m*a*(g3) + hcg*m*(g1 + Vdot) + mvdot2*Vdot )/L

		A = (g)**2 + (c)**2 - mu**2*(e)**2
		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f))
		C = (h)**2 + (d)**2 - mu**2*(f)**2

		for i in range(len(B)):
			try:
				VmaxRear[i] = np.sqrt((-B[i] + np.sqrt(B[i]**2 - 4*A[i]*C[i]))/(2*A[i]))
				if np.isnan(VmaxRear[i]):
					VmaxRear[i] = self.vMax #handle negative square roots
			except:
				VmaxRear[i] = self.vMax


		Vmax = np.minimum(VmaxFront,VmaxRear)
		Vmax = np.minimum(Vmax,self.vMax)

		return Vmax, Vdot

	def generateSpeedProfile(self, speedLim, AxDesired,mu,sigma,Fv2,G,Mv2,Mvdot,theta):


		n = len(sigma)
		# Integrate backwards from minimum speed point to find safe upper bound on
		# speed accounting for turning limits and deceleration limits

		IDX = np.argmin(speedLim)

		pos = IDX+1 % n

		if pos != 0:
			step = sigma[1] - sigma[0]
			muConcat = np.concatenate((mu[pos:], mu[:pos])) #circshift should really be used here
			sigmaConcat = np.concatenate((sigma[pos:], sigma[:pos])) #circshift should really be used here
			fv2Concat = np.concatenate((Fv2[:, pos:], Fv2[:,:pos]), axis = 1) #circshift should really be used here
			gConcat =   np.concatenate((G[:,pos:], G[:,:pos]), axis = 1) #circshift should really be used here
			mv2Concat = np.concatenate((Mv2[1:,pos:], Mv2[1:,:pos]), axis = 1) #circshift should really be used here
			mvdotConcat =    np.concatenate(( Mvdot[1:,pos:], Mvdot[1:,:pos]), axis = 1) #circshift should really be used here
			thetaConcat =    np.concatenate(( theta[pos:], theta[:pos])) #circshift should really be used here
			speedLimConcat = np.concatenate(( speedLim[pos:], speedLim[:pos])) #circshift should really be used here

			posDesired,UxDesired, AxDesired  = self.integrateBackward(step,n,speedLim[IDX],AxDesired[IDX],muConcat , sigmaConcat , fv2Concat ,gConcat ,mv2Concat, mvdotConcat ,thetaConcat, speedLimConcat)
		else:
			posDesired,UxDesired, AxDesired  = self.integrateBackward(step,n,speedLim[IDX],AxDesired[IDX],mu,              sigma,              Fv2,             G,             Mv2[1:,:],      Mvdot[1:,:],      theta,             speedLim)
		
		# return points to normal order
		I = np.argsort(posDesired, axis = 0)

		posDesired = posDesired[I].squeeze()
		UxDesired = UxDesired[I].squeeze()
		AxDesired = AxDesired[I].squeeze()

		# Integrate forwards accounting for friction limits
		pos = IDX % n
		

		if pos != 0:

			posConcat = np.concatenate((posDesired[pos:], posDesired[:pos]))
			UxConcat = np.concatenate((UxDesired[pos:], UxDesired[:pos]))
			AxConcat = np.concatenate((AxDesired[pos:], AxDesired[:pos]))
			muConcat = np.concatenate((mu[pos:], mu[:pos]))
			fv2Concat = np.concatenate((Fv2[:, pos:], Fv2[:,:pos]), axis = 1) #circshift should really be used here
			gConcat =   np.concatenate((G[:,pos:], G[:,:pos]), axis = 1) #circshift should really be used here
			mv2Concat = np.concatenate((Mv2[1:,pos:], Mv2[1:,:pos]), axis = 1) #circshift should really be used here
			mvdotConcat =    np.concatenate(( Mvdot[1:,pos:], Mvdot[1:,:pos]), axis = 1) #circshift should really be used here
			thetaConcat =    np.concatenate(( theta[pos:], theta[:pos])) #circshift should really be used here

			posDesired,UxDesired, AxDesired, AxMax = self.integrateForward(step,posConcat,  UxConcat,   AxConcat,  muConcat, fv2Concat,gConcat,mv2Concat,mvdotConcat,thetaConcat)
		
		else:
			posDesired,UxDesired, AxDesired, AxMax = self.integrateForward(step,posDesired, UxDesired, AxDesired,  mu,       Fv2,      G,      Mv2,      Mvdot,       theta)
		


		# sort back into order from start to  of the path
		I = np.argsort(posDesired, axis = 0)
		posDesired = posDesired[I]
		UxDesired = UxDesired[I]
		AxDesired = AxDesired[I]
		AxMax = AxMax[I].squeeze() #make same dimension as others

		return posDesired, UxDesired, AxDesired, AxMax

	def integrateBackward(self,step, n, endSpeed, ax, mu, sigma, fv2, g, mv2, mvdot, theta, speedLimit):

		positions = sigma.reshape((n,1))  # distance along the path
		Ux = np.zeros((n,1))    # speed [m/s]
		Ax = np.zeros((n,1))    # acceleration [m/s**2]

		Vsquared = endSpeed**2
		beta = self.vehicle.beta

		for i in range(0, n-1):
			Ux[n-i-1] = np.sqrt(Vsquared)
			Ax[n-i-1] = ax

			# find limits on front and rear tires and max speed
			axF,betaF = self.decelFront(Vsquared,g[:,n-i-1],fv2[:,n-i-1],mv2[:,n-i-1],mvdot[:,n-i-1],mu[n-i-1], beta)
			axR,betaR = self.decelRear (Vsquared,g[:,n-i-1],fv2[:,n-i-1],mv2[:,n-i-1],mvdot[:,n-i-1],mu[n-i-1], beta)
			Vmax = speedLimit[n-i-2]
			
			# take smaller magnitude and corresponding brake proportioning
			if axF > axR:
				ax = axF
				beta = betaF
			else:
				ax = axR
				beta = betaR
			
			ax = min(ax,0)

			# control how fast we can come off the brakes to limit understeer on
			# entry
			if (ax-Ax[n-i-1])/step*np.sqrt(Vsquared) < -self.jerkLimit:
				ax = -self.jerkLimit*step/np.sqrt(Vsquared) + Ax[n-i-1]
			
			
			# update
			Vsquared = Vsquared - 2*step*ax/cos(theta[n-i-1])
			if (Vsquared < 0): 
				Vsquared = 0
				ax = 0
			
			if Vsquared > Vmax**2:
				Vsquared = Vmax**2
				ax = 0
			

		Ux[0] = np.sqrt(Vsquared)
		Ax[0] = ax    	
		return positions, Ux, Ax

	def integrateForward(self, step, posDesired, UxDesired, AxDesired, mu, fv2, g, mv2, mvdot, theta):
		
		n = len(posDesired)
		Vsquared = UxDesired[0]**2
		AxMax = np.zeros((n,1))

		P = self.vehicle.powerLimit # engine power, [W]
		m = self.vehicle.m
		D = self.vehicle.dragCoeff
		rr = self.vehicle.rollResistance
		beta = self.vehicle.beta

		for i in range(n-1):
			UxDesired[i] = np.sqrt(Vsquared)

			# find limits on front and rear tires and max speed
			axF, betaF = self.accelFront(Vsquared,g[:,i],fv2[:,i],mv2[:,i], mvdot[:,i],mu[i], beta)
			axR, betaR = self.accelRear (Vsquared,g[:,i],fv2[:,i],mv2[:,i], mvdot[:,i],mu[i], beta)

			# take smaller magnitude and corresponding torque proportioning
			if (axF > axR):
				ax = axR
				beta = betaR
			else:
				ax = axF
				beta = betaF
			

			AxMax[i] = ax     # save for later

			ax = min(ax,P/m/np.sqrt(Vsquared)-D*Vsquared/m -g[0,i])-rr/m

			# update and avoid royally screwing up
			Vsquared = Vsquared + 2*step*ax/cos(theta[i])

			if Vsquared < 0:
				Vsquared = 0
				ax = 0
			

			# avoid overshooting your brakepoints
			if i < n:
				if np.sqrt(Vsquared) > UxDesired[i+1]:
					Vsquared = UxDesired[i+1]**2
					ax = (UxDesired[i+1]**2 - UxDesired[i]**2)/(2*(posDesired[i+1] - posDesired[i]))
				
			else:
				if np.sqrt(Vsquared) > UxDesired[0]:
					Vsquared = UxDesired[0]**2
					ax = (UxDesired[0]**2 - UxDesired[i]**2)/(2*(posDesired[0] - posDesired[i]))

			AxDesired[i] = ax

		return posDesired, UxDesired, AxDesired, AxMax

	def accelFront(self, Vsquared, g, fv2, mv2, mvdot, mu, beta):

		a = self.vehicle.a
		b = self.vehicle.b
		hcg = self.vehicle.h
		D = self.vehicle.dragCoeff
		L = a+b
		m = self.vehicle.m

		# terms in constraint equation
		g1 = g[0]
		g2 = g[1]
		g3 = g[2]
		fv22 = fv2[1]
		fv23 = fv2[2]
		mv22 = mv2[0]
		mv23 = mv2[1]
		mvdot2 = mvdot[0]
		mvdot3 = mvdot[1]

		# find terms for vdot equation
		g = beta*m/(1 + beta)
		h = beta*(m*g1 + D*Vsquared)/(1+beta)
		c = mvdot3/L
		d = (b*m*(fv22*Vsquared + g2) + mv23*Vsquared)/L
		e = -(m*hcg + mvdot2)/L
		f = (m*b*(fv23*Vsquared+g3) - hcg*(m*g1 + D*Vsquared) - mv22*Vsquared)/L
		A = (g)**2 + (c)**2 - mu**2*(e)**2
		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f))
		C = (h)**2 + (d)**2 - mu**2*(f)**2

		# use positive solution to quadratic eqn.
		vdot = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
		if np.isnan(vdot) or vdot < 0:
		 	vdot = 0 # maybe this is feasible by a different torque dist.
		
		# find the front/rear weight distribution for brake proportioning next time
		Fzf = e*vdot + f
		Fzr = m*(fv23*Vsquared + g3) - Fzf
		beta = Fzf/Fzr
		if (beta < 1):
			beta = 1

		return vdot, beta

	def accelRear(self,Vsquared, g, fv2, mv2, mvdot, mu, beta):
		a = self.vehicle.a
		b = self.vehicle.b
		hcg = self.vehicle.h
		D = self.vehicle.dragCoeff
		L = a+b
		m = self.vehicle.m

		# find maximum acceleration assuming back wheels limit
		g1 = g[0]
		g2 = g[1]
		g3 = g[2]
		fv22 = fv2[1]
		fv23 = fv2[2]
		mv22 = mv2[0]
		mv23 = mv2[1]
		mvdot2 = mvdot[0]
		mvdot3 = mvdot[1]

		g = m/(1 + beta)
		h = (m*g1 + D*Vsquared)/(1+beta)
		c = -mvdot3/L
		d = (a*m*(fv22*Vsquared + g2) - mv23*Vsquared)/L
		e = (m*hcg + mvdot2)/L
		f = (m*a*(fv23*Vsquared+g3) + hcg*(m*g1 + D*Vsquared) + mv22*Vsquared)/L
		A = (g)**2 + (c)**2 - mu**2*(e)**2
		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f))
		C = (h)**2 + (d)**2 - mu**2*(f)**2


		# use positive solution to quadratic eqn.
		vdot = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
		if np.isnan(vdot) or vdot < 0:
			vdot = 0 # maybe this is feasible by a different torque dist.

		Fzr = e*vdot + f
		Fzf = m*(fv23*Vsquared + g3) - Fzr
		beta = Fzf/Fzr
		if (beta < 1):
			beta = 1
		

		return vdot, beta

	def decelFront(self, Vsquared, g, fv2, mv2, mvdot, mu, beta):
		a = self.vehicle.a
		b = self.vehicle.b
		hcg = self.vehicle.h
		D = self.vehicle.dragCoeff
		L = a+b
		m = self.vehicle.m

		#terms in constraint equation
		g1 = g[0]
		g2 = g[1]
		g3 = g[2]
		fv22 = fv2[1]
		fv23 = fv2[2]
		mv22 = mv2[0]
		mv23 = mv2[1]
		mvdot2 = mvdot[0]
		mvdot3 = mvdot[1]

		# find terms for vdot equation
		g = beta*m/(1 + beta)
		h = beta*(m*g1 + D*Vsquared)/(1+beta)
		c = mvdot3/L
		d = (b*m*(fv22*Vsquared + g2) + mv23*Vsquared)/L
		e = -(m*hcg + mvdot2)/L
		f = (m*b*(fv23*Vsquared+g3) - hcg*(m*g1 + D*Vsquared) - mv22*Vsquared)/L
		A = (g)**2 + (c)**2 - mu**2*(e)**2
		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f))
		C = (h)**2 + (d)**2 - mu**2*(f)**2

		#pdb.set_trace()

		# use negative solution to quadratic eqn.
		
		vdot = (-B - np.sqrt(B**2 - 4*A*C))/(2*A) # * brakeFactor # scale deceleration by brakeFactor
		if np.isnan(vdot) or vdot > 0:
			vdot = -h/g   # if limits exceeded, grade and drag only
		
		# find the front/rear weight distribution for brake proportioning next time
		Fzf = e*vdot + f
		Fzr = m*(fv23*Vsquared + g3) - Fzf
		beta = Fzf/Fzr
		vdot = vdot * self.vehicle.brakeFactor    	

		return vdot, beta

	def decelRear(self, Vsquared, g, fv2, mv2, mvdot, mu,  beta):
		a = self.vehicle.a
		b = self.vehicle.b
		hcg = self.vehicle.h
		D = self.vehicle.dragCoeff
		L = a+b
		m = self.vehicle.m

		# find maximum deceleration assuming back wheels limit
		g1 = g[0]
		g2 = g[1]
		g3 = g[2]
		fv22 = fv2[1]
		fv23 = fv2[2]
		mv22 = mv2[0]
		mv23 = mv2[1]
		mvdot2 = mvdot[0]
		mvdot3 = mvdot[1]

		g = m/(1 + beta)
		h = (m*g1 + D*Vsquared)/(1+beta)
		c = -mvdot3/L
		d = (a*m*(fv22*Vsquared + g2) - mv23*Vsquared)/L
		e = (m*hcg + mvdot2)/L
		f = (m*a*(fv23*Vsquared+g3) + hcg*(m*g1 + D*Vsquared) + mv22*Vsquared)/L
		A = (g)**2 + (c)**2 - mu**2*(e)**2
		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f))
		C = (h)**2 + (d)**2 - mu**2*(f)**2


		# use negative solution
		vdot = (-B - np.sqrt(B**2 - 4*A*C))/(2*A)
		if np.isnan(vdot) or (vdot > 0):
			vdot = -h/g   # if limits exceeded, grade and drag only
		
		Fzr = e*vdot + f
		Fzf = m*(fv23*Vsquared + g3) - Fzr
		beta = Fzf/Fzr
		vdot = vdot*self.vehicle.brakeFactor	

		return vdot, beta


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
		#RotaryDerivatives Calculates coefficients for V**2 and Vdot in the moment equation
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
		













