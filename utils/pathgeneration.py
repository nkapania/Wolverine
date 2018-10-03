import numpy as np 
import cvxpy as cp
import utils
from velocityprofiles import *
from simulation import *
from control import *
import sys
from scipy import interpolate

#Implements rapid path generation algorithm described in N. Kapania et al.
#2016 JDSMC paper

class OptimizationResults:
	def __init__(self, vp, logFile, opt, path):
		self.vps = [vp]
		self.logFiles = [logFile]
		self.opt = [opt]
		self.path = [path]




class RapidPathGeneration:
	def __init__(self, vehicle, path, bounds, NUM_ITERS = 5, buff = None): 
		
		#optimization parameters
		self.NUM_ITERS = NUM_ITERS
		self.NUM_POINTS = 1000 #number of points in optimization routine
		self.lam1 = 1 #weight on steering regularization
		self.lam2 = np.zeros((self.NUM_POINTS, 1)) #weight on minimum distance


		#problem data
		self.vehicle = vehicle
		self.initialPath = path #used as the reference for updating the world after every iteration
		self.path = path
		self.bounds = bounds
		self.mu = vehicle.muF * 0.95 #drive at the limits
		
		#add in option to add road buffer
		if buff is None:
			self.buff = ([0, path.s[-1]], [0, 0]) #no road buffer 
		else:
			self.buff = buff
		
		self.widthLeft, self.widthRight  = self.getLaneWidth()
		#initial solution
		self.vp = RacingProfile(vehicle, path, self.mu)
		controller = LaneKeepingController(path, vehicle, self.vp)
		bikeSim = Simulation(vehicle, controller, path = self.path, profile = self.vp, mapMatchType = "closest")
		#logFile0 = bikeSim.simulate()
		#lapTime = bikeSim.getLapTime()
		lapTime = 0

		#append data from iteration 0 into results class
		#self.optResults = OptimizationResults(self.vp, logFile0, None, path)
		self.optResults = OptimizationResults(self.vp, None, None, path)
		self.lapTimes = [lapTime]


	def optimize(self):
		print("Optimizing Path")

		for i in range(self.NUM_ITERS):
			#solve for optimal path
			opt = self.getRapidTrajectory()
			self.updateWorld(opt)
			
			#generate velocity profile
			self.vp = RacingProfile(self.path, self.vehicle, self.mu)

			#simulate and collect new lap time
			bikeSim.updatePath(self.path)
			logFile = bikeSim.simulate()
			lapTime = getLapTime(logFile)

			#cache problem data
			optResults.append(self.vp, logFile, opt, self.path)

		return self.path, self.vp


	def getRapidTrajectory(self):
		#resample world and velocity profile objects for optimization
		ds = self.path.s[-1] / self.NUM_POINTS
		self.path.resample(ds)
		
		self.vp.resample(ds)
		opt = self.getCurvatureProfile()
		return opt


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
		A, B, d = utils.getAllSys(veh, Ux, K, ts)

		#Construct the problem here
		print('Solving Convex Problem ...')

		delta = cp.Variable(n)
		x     = cp.Variable(4,n) 

		#STATE: x[0] is e, x[1] is dPsi, x[2] is yaw rate, x[3] is sideslip

		#add in objectives
		objective = cp.Minimize(cp.norm( 1/ds*(psiR[1:N] - psiR[0:N-1] + x[1,1:N].T - x[1,0:N-1].T), 2) + lam1*norm(delta[1:N]-delta[0:N-1]) + cp.sum(  lam2[0:N-1]*(ds/Ux[0:N-1]* ( -K[0:N-1]*x[0,0:N-1].T  )   +   ds/Ux[0:N-1]*cp.square( x[3,0:N-1] + x[1,0:N-1]   ).T)  ))
		

		#add in constraints
		constraints = []

		for i in range(n-1):
			constraints += [ x[:,i+1] == A[i]*x[:,i] + B[i]*delta[i] + d[i] ]

		constraints += [x[0,:] <= self.widthLeft]
		constraints += [x[0,:] >= self.widthRight]


		constraints += [x[0,n] == x[0,0]                          - offset] #account for any small offsets
		constraints += [x[1,n] == x[1,0]                                  ] #continuity for dPsi, beta, and r between beginning and end.
		constraints += [x[2,n] == x[2,0]                                  ]
		constraints += [x[3,n] == x[3,0]                                  ]
		 
		constraints += [(x[0,n] - x[0,n-1])/ds[-1] == (x[0,1] - x[0,0])/ds[0] ]#also needed to ensure dPsi continuity - still not sure why though.

		prob = cp.Problem(objective, constraints)
		res =prob.solve()

		opt = {}
		opt["aF"] = x[3,:] + veh.a*x[2,:]/Ux.T - delta.T
		opt["aR"] = x[3,:] - veh.b*x[2,:]/Ux.T
		opt["e"] = x[0,:]
		opt["dPsi"] = x[1,:]
		opt["r"] = x[2,:]
		opt["roadPsi"] = opt.dPsi + psiR
		opt["beta"] = x[3,:].T 
		opt["s"] = self.path.s
		opt["ts"] = ts
		opt["delta"] = delta
		[opt.posE, opt.posN] = convertPathToGlobal(self.path, self.path.s, opt["e"]) 
		opt["K"] = 1/ds*np.diff(psiR + opt["dPsi"])
		opt["K"] = np.concatenate(opt["K"][0], opt["K"]) #ensure consistency of dimensions
		opt.feasible = res != np.inf
		if not opt.feasible:
			sys.exit('No feasible solution found')

		return opt


	def updateWorld(self, opt):

		tck = interpolate.splrep(opt["s"], opt["e"])
		e   = interpolate.splev(self.initialPath, tck)

		posE, posN = convertPathToGlobal(refWorld, refWorld.s, e); 
		
		#generate a new world object from the EN point cloud
		self.path = genWorldFromEN(posE, posN)
		
		#update buffer and lam2
		self.updateBuffer()
		self.updateLam2()

		#recompute lane width 
		world = self.getLaneWidth()

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
	    	#print(widthLeft[i])
	    	#print(idxLeft)

	    for i in range(n):
	    	point = [path.posE[i], path.posN[i]]
	    	widthRight[i], idxRight = getMinDistance(point, self.bounds["in"], idxRight) - buffer[i]

	    self.widthLeft = widthLeft
	    self.widthRight = widthRight

	    return widthLeft, widthRight






#########################################################################################
######################### HELPER FUNCTIONS ##############################################
#########################################################################################



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
				lastInd = idx

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
				lastInd = idx

			if idx is 0:
				idx = n - 1

			else:
				idx = idx - 1

			i = i + 1

	return shortest, idx










































