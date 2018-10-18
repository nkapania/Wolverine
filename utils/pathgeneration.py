import numpy as np 
import cvxpy as cp
import utils
from velocityprofiles import *
from simulation import *
from control import *
from paths import *
import sys
from scipy import interpolate
import pdb

#Implements rapid path generation algorithm described in N. Kapania et al.
#2016 JDSMC paper

class OptimizationResults:
	def __init__(self, vpDict, logFile, optDict, pathDict, lapTime):
		self.vps = [vpDict]
		self.logFiles = [logFile]
		self.opt = [optDict]
		self.path = [pathDict]
		self.lapTimes= [lapTime]

	def append(self, vpDict, logFile, optDict, pathDict, lapTime):
		self.vps.append(vpDict)
		self.logFiles.append(logFile)
		self.opt.append(optDict)
		self.path.append(pathDict)
		self.lapTimes.append(lapTime)


class RapidPathGeneration:
	def __init__(self, vehicle, path, bounds, mu, NUM_ITERS = 5, buff = None): 
		
		#optimization parameters
		self.NUM_ITERS = NUM_ITERS
		self.NUM_POINTS = 1863 #number of points in optimization routine - currently set to match MATLAB
		self.lam1 = 1.0 #weight on steering regularization
		self.lam2 = np.zeros((self.NUM_POINTS, 1)) #weights on minimum distance


		#problem data
		self.vehicle = vehicle
		self.initialPath = path #used as the reference for updating the world after every iteration
		self.path = path
		self.bounds = bounds
		self.mu = mu
		
		#add in option to add road buffer
		if buff is None:
			self.buff = ([0, path.s[-1]], [0, 0]) #no road buffer 
		else:
			self.buff = buff
		
		self.widthLeft, self.widthRight  = self.getLaneWidth()
		#initial solution
		self.vp = RacingProfile(vehicle, path, self.mu, vMax = 99)
		self.controller = LaneKeepingController(path, vehicle, self.vp)
		bikeSim = Simulation(vehicle, self.controller, path = self.path, profile = self.vp, mapMatchType = "closest")
		logFile0 = bikeSim.simulate()
		lapTime = bikeSim.getLapTime()
		
		#append data from iteration 0 into results class
		self.optResults = OptimizationResults(self.vp.toDict(), logFile0, None, self.initialPath.toDict(), lapTime)
		

	def optimize(self):
		print("Optimizing Path")

		for i in range(self.NUM_ITERS):
			#solve for optimal path
			opt = self.getRapidTrajectory()
			self.updateWorld(opt)
			
			#generate velocity profile
			self.vp = RacingProfile(self.vehicle, self.path, self.mu)

			#simulate and collect new lap time

			#note - generally not great to initialize objects within loop, but N here is less than 5 so probably OK
			bikeSim = Simulation(self.vehicle, self.controller, path = self.path, profile = self.vp, mapMatchType = "closest")
			

			logFile = bikeSim.simulate()
			lapTime = bikeSim.getLapTime()

			#cache problem data
			self.optResults.append(self.vp.toDict(), logFile, opt, self.path.toDict(), lapTime)

		return self.optResults


	def getRapidTrajectory(self):
		#resample world and velocity profile objects for optimization
		ds = self.path.s[-1] / (self.NUM_POINTS)
		
		self.path.resample(ds)	
		self.vp.resample(ds)
		self.resampleLaneWidth()

		opt = self.getCurvatureProfile()

		return opt


	def resampleLaneWidth(self):
		#called after path and velocity profile are resampled

		n = len(self.path.s)
		N = len(self.widthLeft)

		ind = np.round(np.linspace(0, N-1, n))
		ind = ind.astype(int)
		
		self.widthLeft = self.widthLeft[ind]
		self.widthRight= self.widthRight[ind]

		return


	def getCurvatureProfile(self):
		
		#unpack parameters
		n = len(self.path.s)
		assert n == self.NUM_POINTS
		s = self.path.s
		K = self.path.curvature 
		Ux = self.vp.Ux
		psiR = self.path.roadPsi
		lam1 = self.lam1
		lam2 = self.lam2

		#get problem data

		offset = np.linalg.norm( [self.path.posE[0] - self.path.posE[-1], self.path.posN[0] - self.path.posN[-1]] )
		ds = np.diff(s)

		_, ts = getLapTime(self.vp.s, self.vp.Ux)

		print('Generating Affine Tire Models ...')
		A, B, d = utils.getAllSys(self.vehicle, Ux, K, ts)

		#Construct the problem here
		print('Solving Convex Problem ...')

		delta = cp.Variable(n)
		x     = cp.Variable((4,n)) 

		#STATE: x[0] is e, x[1] is dPsi, x[2] is yaw rate, x[3] is sideslip
		Kmat = np.diag(-K[0:n-1])

		objective = cp.Minimize(cp.norm( 1/ds*(psiR[1:n] - psiR[0:n-1] + x[1,1:n].T - x[1,0:n-1].T), 2) +
		lam1*cp.norm(delta[1:n]-delta[0:n-1]) + 
		cp.sum(  lam2[0:n-1]*(ds/Ux[0:n-1]* ( Kmat*x[0,0:n-1].T  )   +
		ds/Ux[0:n-1]*cp.square( x[3,0:n-1] + x[1,0:n-1]   ).T) ))

		#add in constraints
		constraints = []

		for i in range(n-1):
			constraints += [ x[:,i+1] == A[i]*x[:,i] + B[i]*delta[i] + d[i] ]

		constraints += [x[0,:] <= self.widthLeft.squeeze()]
		constraints += [x[0,:] >= self.widthRight.squeeze()]


		constraints += [x[0,n-1] == x[0,0]                          - offset] #account for any small offsets
		constraints += [x[1,n-1] == x[1,0]                                  ] #continuity for dPsi, beta, and r between beginning and end.
		constraints += [x[2,n-1] == x[2,0]                                  ]
		constraints += [x[3,n-1] == x[3,0]                                  ]
		 
		constraints += [(x[0,n-1] - x[0,n-2])/ds[-1] == (x[0,1] - x[0,0])/ds[0] ]#also needed to ensure dPsi continuity - still not sure why though.

		prob = cp.Problem(objective, constraints)
		res =prob.solve()

		x = x.value
		opt = {}
		opt["feasible"] = res != np.inf
		if not opt["feasible"]:
			sys.exit('No feasible solution found')
		opt["aF"] = x[3,:] + self.vehicle.a*x[2,:]/Ux - delta
		opt["aR"] = x[3,:] - self.vehicle.b*x[2,:]/Ux.T
		opt["e"] = x[0,:]
		opt["dPsi"] = x[1,:]
		opt["r"] = x[2,:]
		opt["roadPsi"] = opt["dPsi"] + psiR
		opt["beta"] = x[3,:].T 
		opt["s"] = self.path.s
		opt["widthLeft"] = self.widthLeft
		opt["widthRight"] = self.widthRight
		opt["ts"] = ts
		opt["delta"] = delta
		opt["posE"], opt["posN"] = convertPathToGlobal(self.path, self.path.s, opt["e"]) 
		opt["K"] = 1/ds*np.diff(psiR + opt["dPsi"])
		opt["K"] = np.concatenate((opt["K"][0, np.newaxis], opt["K"])) #ensure consistency of dimensions

		return opt


	def updateWorld(self, opt):

		tck = interpolate.splrep(opt["s"], opt["e"])
		e   = interpolate.splev(self.initialPath.s, tck)

		posE, posN = convertPathToGlobal(self.initialPath, self.initialPath.s, e); 
		
		#generate a new world object from the EN point cloud
		self.path = Path() 
		self.path.genFromEN(posE, posN, isOpen = False) #closed map
		
		#update buffer and lam2
		#self.updateBuffer()
		#self.updateLam2()

		#recompute lane width 
		_,_ = self.getLaneWidth()

		return

	def getLaneWidth(self):
	    print('Getting Lane Boundaries')

	    path = self.path
	    n = len(path.s)
	    widthLeft = np.zeros((n,1))
	    widthRight = np.zeros((n,1))
	    buffS, buffVal = self.buff

	    buffer = np.interp(path.s, buffS, buffVal)


	    idxLeft = -1
	    idxRight = -1

	    for i in range(n):
	    	point = [path.posE[i], path.posN[i]]

	    	widthLeft[i], idxLeft = getMinDistance(point, self.bounds["in"], idxLeft) - buffer[i]

	    for i in range(n):
	    	point = [path.posE[i], path.posN[i]]
	    	widthRight[i], idxRight = getMinDistance(point, self.bounds["out"], idxRight) - buffer[i]

	    self.widthLeft = widthLeft
	    self.widthRight = -widthRight

	    return widthLeft, -widthRight






#########################################################################################
######################### HELPER FUNCTIONS ##############################################
#########################################################################################


def convertPathToGlobal(path, s, e):
	#converts s and e vectors along a path defined by world into S and E coordinates
	n = len(s)
	E = np.zeros((n,1))
	N = np.zeros((n,1))

	centE = np.interp(s, path.s, path.posE)
	centN = np.interp(s, path.s, path.posN)
	theta = np.interp(s, path.s, path.roadPsi)

	for i in range(n):
		E[i] = centE[i] - e[i] * np.sin( np.pi / 2 - theta[i])
		N[i] = centN[i] - e[i] * np.cos( np.pi / 2 - theta[i])


	return E, N

#gets minimum distance between a point and a point cloud representing a road boundary or racing line
#Can make this more sophisticated for a speedup
def getMinDistance(point, line, lastidx):

	lastidx = int(lastidx) #for some reason idx becomes a non-integer

	x = line[:,0]
	n = len(x)
	N_UP = 15
	N_DOWN = 15

	shortest = np.inf



	#search the whole map
	if lastidx == -1:
		for i in range(n): 
			dist = np.linalg.norm(point - line[i,:])
			if dist < shortest:
				shortest = dist
				lastidx = i

		return shortest, lastidx

	else:
		idx = lastidx
		i = 0

		while i < N_UP:
			dist = np.linalg.norm(point - line[idx,:])
			if dist < shortest:
				shortest = dist
				lastidx = idx

			if idx == (n-1):
				idx = 0
			else:
				idx = idx + 1

			i = i + 1


		i = 0
		idx = lastidx

		while i < N_DOWN:
			dist = np.linalg.norm(point - line[idx,:])
			if dist < shortest:
				shortest = dist
				lastidx = idx

			if idx is 0:
				idx = n - 1

			else:
				idx = idx - 1

			i = i + 1

		
	return shortest, lastidx










































