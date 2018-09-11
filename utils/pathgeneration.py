import numpy as np 
import cvxpy as cp
from velocityprofiles import *
from simulation import *
from control import *

#Implements rapid path generation algorithm described in N. Kapania et al.
#2016 JDSMC paper
class RapidPathGeneration:
	def __init__(self, vehicle, path, bounds, NUM_ITERS = 5, buff = None): 
		
		#optimization parameters
		self.NUM_ITERS = NUM_ITERS
		self.NUM_POINTS = 1000 #number of points in optimization routine
		self.lam1 = 1 #weight on steering regularization
		self.lam2 = np.zeros((NUM_POINTS, 1))


		#problem data
		self.vehicle = vehicle
		self.path = path
		self.bounds = bounds
		self.mu = vehicle.muF * 0.95 #drive at the limits
		
		#add in option to add road buffer
		if buff is None:
			self.buff = ([0, path.s[-1]], [0, 0]) #no road buffer 
		else:
			self.buff = buff
		
		#initial solution
		self.vp = RacingProfile(vehicle, path, self.mu)
		controller = LaneKeepingController()
		bikeSim = Simulation(vehicle, controller, path = self.path, profile = self.vp, mapMatchType = "closest")
		logFile0 = bikeSim.simulate()
		lapTime = getLapTime(logFile0)

		#append data from iteration 0 into results class
		self.optResults = OptimizationResults(self.vp, logFile0, None, path)
		self.lapTimes = [lapTime]


	def optimize(self):
		print("Optimizing Path")

		for i in range(NUM_ITERS):
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


	def getRapidTrajectory(self):
		#resample world and velocity profile objects for optimization
		ds = self.path.s[-1] / self.NUM_POINTS
		self.path.resample(ds)
		
		self.vp.resample(ds)
		opt = self.getCurvatureProfile()


	def getCurvatureProfile(self):
		n = len(self.path.s)
		assert n == self.NUM_POINTS

		#unpack parameters
		s = self.path.s
		K = self.path.curvature 
		Ux = self.vp.Ux
		psiR = self.path.roadPsi

		offset = np.norm( [self.path.posE[0] - self.path.posE[-1], self.path.posN[0] - self.path.posN[-1]] )
		ds = np.diff(s)

		_, ts = vp.getLapTime()

		print('Generating Affine Tire Models ...')
		A, B, d = self.getAllSys(veh, Ux, K, ts)

		print('Solving Convex Problem ...')
		#Construct the problem here

		delta = cp.Variable(n)
		x     = cp.Variable(4,n)

		minimize lam0*norm(






		# m = 30
		# n = 20
		# np.random.seed(1)
		# A = np.random.randn(m, n)
		# b = np.random.randn(m)

		# # Construct the problem.
		# x = cp.Variable(n)
		# objective = cp.Minimize(cp.sum_squares(A*x - b))
		# constraints = [0 <= x, x <= 1]
		# prob = cp.Problem(objective, constraints)

		# # The optimal objective value is returned by `prob.solve()`.
		# result = prob.solve()
		# # The optimal value for x is stored in `x.value`.
		# print(x.value)
		# # The optimal Lagrange multiplier for a constraint is stored in
		# # `constraint.dual_value`.
		# print(constraints[0].dual_value)
























