import numpy as np
import matplotlib.pyplot as plt
import tiremodels as tm
import vehicles
import paths
import scipy.io as sio
from control import *
import time
import os.path
from datetime import datetime

#Defines a simulation class and a state class

class Simulation:
	def __init__(self, vehicle, controller, path = None, profile = None, mapMatchType = "euler",
		maxTime = float('inf'), vStart = 10., weightTransferType = None,
		tires = "fiala", desiredLaps = 1): 
		
		self.path = path
		self.vehicle = vehicle
		self.profile = profile
		self.controller = controller
		self.isRunning = True
		self.physics = "bicycle"
		self.logFile = {}
		self.ts = 0.01 #simulation time in seconds
		self.mapMatchType = mapMatchType
		self.maxTime = maxTime
		self.vStart = vStart
		self.weightTransferType = weightTransferType
		self.tires = tires
		self.desiredLaps = desiredLaps
		self.lapNumber = 0 #start at lap 0
		self.last_s = 0. #last value of s, used to determine when a lap has switched
		
	def simulate(self):
		##initialize states and instantiate objects

		if self.profile is None:
			Ux0 = self.vStart  #specify start state
		else:
			Ux0 = self.profile.Ux[0] #start car at the initial velocity specified in the path

		localState = LocalState(Ux0)
		globalState = GlobalState(self.path)
		controlInput = ControlInput()
		log = Logger()
		mapMatch = MapMatch(self.path, self.mapMatchType)


		##Start the Counter
		counter = 0

		##Run the simulation!
		while self.isRunning:
			
			#Perform localization
			mapMatch.localize(localState, globalState)

			#Check to see if we should terminate
			self.checkForTermination(localState, counter, log)

			#Calculate controller inputs
			auxVars = self.controller.updateInput(localState, controlInput)            

			#Update state
			derivs, slips, forces = self.updateState(controlInput, localState, globalState, auxVars)

			#Append counter and print to screen
			counter = counter + 1
			self.printStatus(localState, counter)


			#save signals needed
			AxDes = auxVars["AxDes"]
			UxDes = auxVars["UxDes"]                
			log.append('t',counter*self.ts)
			log.append('K', auxVars["K"])
			log.append('alphaFdes', auxVars["alphaFdes"])
			log.append('alphaRdes', auxVars["alphaRdes"])
			log.append('deltaFFW',  auxVars["deltaFFW"])
			log.append('deltaFB' ,  auxVars["deltaFB"])
			log.append('betaFFW' ,  auxVars["betaFFW"])
			log.append('Ux',localState.Ux)
			log.append('Ax', derivs["Ax"])
			log.append('s', localState.s)
			log.append('e', localState.e)
			log.append('lapNumber', self.lapNumber)
			log.append('UxDes', UxDes) 
			log.append('AxDes', AxDes)
			log.append('deltaPsi', localState.deltaPsi)
			log.append('posE', globalState.posE)
			log.append('posN', globalState.posN)
			log.append('psi', globalState.psi)
			log.append('r', localState.r)
			log.append('Uy', localState.Uy)
			log.append('deltaCmd', controlInput.delta)
			log.append('FxCmd', controlInput.Fx)
			log.append('alphaF', slips["alphaF"])
			log.append('alphaR', slips["alphaR"])
			log.append('FxF', forces["FxF"])
			log.append('FxR', forces["FxR"])
			log.append('FyF', forces["FyF"])
			log.append('FyR', forces["FyR"])
			log.append('zetaF', forces["zetaF"])
			log.append('zetaR', forces["zetaR"])
			log.append('FzF', forces["FzF"])
			log.append('FzR', forces["FzR"])

			log.incrementCounter()

		self.logFile = log.getData()
		return self.logFile

	def incrementLapNumber(self, s):
		#method to check if we have elapsed a lap
		if self.last_s > s:
			self.lapNumber += 1

		self.last_s = s





	def checkForTermination(self, localState, counter, log):

		#Check if we have ended the simulation. If running open loop, stop after maximum time is specified. Otherwise stop once we reach the end of the track or if we go unstable

		t = counter * self.ts
		lapNumber = self.incrementLapNumber(localState.s)

		if self.path is None:  
			assert (self.maxTime != None), "Must specify time to run"
			if t > self.maxTime:
				self.isRunning = False

		else:
			
			if t > self.maxTime:
				self.isRunning = False

			if localState.s > (self.path.s[-1] - 0.55) and self.lapNumber == self.desiredLaps - 1: #Stop simulation a little before end of path
				self.isRunning = False
				runTime = counter * self.ts
				print("Simulation complete - total time %.2f sec" %runTime)             

			#Check if we have gone off the track    
			if abs(localState.e) > 5.0:
				print("Car has left the track - terminating...")
				self.isRunning = False

	def updatePath(self, path):
		self.path = path
		return

	def updateVelocityProfile(self, profile):
		self.profile = profile
		return


	def printStatus(self, localState, counter):
		if self.path is None:
			t = counter * self.ts 
			pctComplete = np.ceil( 100 * t / self.maxTime)
		else:
			pctComplete = np.ceil( 100 * localState.s / self.path.s[-1] )

		if np.mod(counter, 100) == 0:
			print("Simulation is %02d percent done" % pctComplete)
			#print(pctComplete)
			#print("Distance Along Path is %04d meters") % localState.s

	def updateState(self, controlInput, localState, globalState, auxVars):
		K = auxVars["K"]
		UxDes = auxVars["UxDes"]
		if self.physics is "bicycle":
			derivs, slips, forces = bicycleModel(self.vehicle, controlInput, localState, globalState, self.mapMatchType, self.ts, K, self.weightTransferType, self.tires)
		
		return derivs, slips, forces

	def getLapTime(self):
		return self.logFile["t"][-1]

	def save(self, filename):

		#do not overwrite if path exists
		if os.path.isfile(filename+".mat"):
			print("Warning - filename already found - appending time to filename")
			filename = filename + "_" + str(datetime.now().strftime('%M:%S.%f')[:-4])
		
		sio.savemat(filename, self.logFile)

		print("Simulation results saved to " + filename)



	#plots simulation results        
	def plotResults(self, xaxis = "s"):

		#unpack arrays for plotting
		posE = self.logFile["posE"]
		posN = self.logFile["posN"]
		e = self.logFile["e"]
		dPsi = self.logFile["deltaPsi"]
		s = self.logFile["s"]
		t = self.logFile["t"]
		Ux = self.logFile["Ux"]
		Uy = self.logFile["Uy"]
		Ax = self.logFile["Ax"]
		UxDes = self.logFile["UxDes"]
		AxDes = self.logFile["AxDes"]
		K = self.logFile["K"]
		alphaFdes = self.logFile["alphaFdes"]
		alphaRdes = self.logFile["alphaRdes"]
		alphaF = self.logFile["alphaF"]
		alphaR = self.logFile["alphaR"]
		FyF = self.logFile["FyF"]
		FyR = self.logFile["FyR"]
		FxF = self.logFile["FxF"]
		FxR = self.logFile["FxR"]
		zetaF = self.logFile["zetaF"]
		zetaR = self.logFile["zetaR"]
		deltaFFW = self.logFile["deltaFFW"]
		betaFFW  = self.logFile["betaFFW"]
		deltaFB  = self.logFile["deltaFB"]
		delta = self.logFile["deltaCmd"]
		beta = np.tan(np.divide(Uy,Ux))


		

		#auxillary calculations
		posEdes = self.path.posE
		posNdes = self.path.posN

		if xaxis is "s":
			x = s
			xstr = "s (m)"
		else:
			x = t
			xstr = "t (sec)" 


		#Plot the path and the world
		plt.figure()
		plt.plot(posE, posN,'k', linewidth = 2)
		plt.plot(posEdes, posNdes,'k--', linewidth = 1)
		plt.grid(True)
		plt.axis('equal')
		plt.legend(('Actual','Desired'))
		plt.xlabel('East')
		plt.ylabel('North')


		#plot the tracking errors
		plt.figure()
		ax1=plt.subplot(2, 1, 1)
		ax1.plot(x, e,'k', linewidth = 2)
		ax1.plot(x, dPsi * 180 / np.pi,'r', linewidth = 2)
		plt.grid(True)
		plt.legend(('e (m)','dPsi (deg)'))
		plt.ylim([-5., 5.])
		plt.xlabel(xstr)

		ax2 = plt.subplot(2, 1, 2, sharex = ax1)
		ax2.plot(x, Ux - UxDes,'k', linewidth = 2)
		plt.grid(True)
		plt.ylabel("Speed Error (m/s)")
		plt.xlabel(xstr)

		#plot the velocity profile
		plt.figure()
		ax3 = plt.subplot(3, 1, 1, sharex = ax1)
		ax3.plot(x, Ux,'k', linewidth = 2)
		ax3.plot(x, UxDes,'k--', linewidth = 1)
		plt.grid(True)
		plt.legend(('Actual','Desired'))
		plt.ylabel('Velocity (m/s)')
		plt.xlabel(xstr)

		ax4 = plt.subplot(3, 1, 2, sharex = ax1)
		ax4.plot(x, Ax,'k', linewidth = 2)
		ax4.plot(x, AxDes,'k--', linewidth = 1)
		plt.grid(True)
		plt.legend(('Actual','Desired'))
		plt.ylabel('Acceleration (m/s2)')
		plt.xlabel(xstr)

		ax5 = plt.subplot(3, 1, 3, sharex = ax1)
		ax5.plot(x, K,'k', linewidth = 2)
		plt.grid(True)
		plt.ylabel('Curvature (1/m)')
		plt.xlabel(xstr)

		#plot the tire slips
		plt.figure()
		ax6 = plt.subplot(2, 1, 1, sharex = ax1)
		ax6.plot(x, alphaF * 180 / np.pi,'k', linewidth = 2)
		ax6.plot(x, alphaFdes * 180 / np.pi,'k--', linewidth = 1)
		plt.grid(True)
		plt.legend(('Actual',' Desired'))
		plt.ylabel('Front Tire Slips (deg)')
		plt.xlabel(xstr)

		ax7 = plt.subplot(2, 1, 2, sharex = ax1)
		ax7.plot(x, alphaR * 180 / np.pi, 'k', linewidth = 2)
		ax7.plot(x, alphaRdes * 180 / np.pi, 'k--',linewidth = 1)
		plt.grid(True)
		plt.ylabel('Rear Tire Slips (deg)')

		#plot the tire forces
		plt.figure()
		ax8 = plt.subplot(3, 1, 1, sharex = ax1)
		ax8.plot(x, FyF / 1000,'k', linewidth = 2)
		ax8.plot(x, FyR / 1000,'b', linewidth = 2)
		plt.grid(True)
		plt.legend(('Front',' Rear'))
		plt.ylabel('Lateral Tire Forces (kN)')
		plt.xlabel(xstr)

		ax9 = plt.subplot(3, 1, 2, sharex = ax1)
		ax9.plot(x, FxF / 1000, 'k', linewidth = 2)
		ax9.plot(x, FxR / 1000, 'b',linewidth = 2)
		plt.legend(('Front','Rear'))
		plt.grid(True)
		plt.ylabel('Long. Tire Forces (kN)')

		ax10 = plt.subplot(3, 1, 3, sharex = ax1)
		ax10.plot(x, zetaF, 'k', linewidth = 1)
		ax10.plot(x, zetaR, 'r--',linewidth = 2)
		plt.legend(('Front','Rear'))
		plt.grid(True)
		plt.ylabel('zetas')

		plt.figure()
		ax11 = plt.subplot(2, 1, 1, sharex = ax1)
		ax11.plot(x, beta * 180/np.pi, 'k', linewidth = 1)
		ax11.plot(x, betaFFW * 180/np.pi, 'r',linewidth = 2)
		plt.legend(('beta','betaFFW'))
		plt.grid(True)
		plt.ylabel('Sideslip (deg)')

		ax12 = plt.subplot(2, 1, 2, sharex = ax1)
		ax12.plot(x, delta * 180 / np.pi, 'k', linewidth = 1)
		ax12.plot(x, deltaFFW * 180 / np.pi, 'r--',linewidth = 2)
		ax12.plot(x, deltaFB * 180 / np.pi, 'g--',linewidth = 2)
		plt.legend(('delta','deltaFFW','deltaFB'))
		plt.grid(True)
		plt.ylabel('Steering Command (deg)')

		plt.show()


class MyAnimation:
	def __init__(self, logFile, path, veh, timeStep = 0.01, interval = 25):
		#plot the desired road
		self.f = plt.figure()
		self.logFile = logFile
		self.path = path
		self.timeStep = timeStep
		self.windowSize = 20. #meters
		self.interval = interval
		self.veh = veh

		plt.ion

		plt.plot(self.path.posE, self.path.posN, 'k--',Linewidth = 2.0)
		plt.grid('on')
		plt.axis('equal')
		


	def run(self):

		#unpack variables
		x = self.logFile["posE"]
		y = self.logFile["posN"]
		delta = self.logFile["deltaCmd"]
		psi  = self.logFile["psi"]
		a = self.veh.a
		b = self.veh.b
		d = self.veh.d
		rW = self.veh.rW


		#calculate vehicle coordinates
		FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire = _plotVehicle(x[0], y[0], psi[0], delta[0], a, b, d, rW)


		#plot vehicle coordinates - needs to be done once outside of loop I believe
		l0,  = plt.plot(x[0], y[0], 'ro', MarkerSize=8)

		l1, = plt.plot(FrontBody[0,:]  ,FrontBody[1,:], 'gray', LineWidth = 2.5)
		l2, = plt.plot(RearBody[0,:]   ,RearBody[1,:], 'gray', LineWidth = 2.5) 
		l3, = plt.plot(FrontAxle[0,:]  ,FrontAxle[1,:], 'gray', LineWidth = 2.5)
		l4, = plt.plot(RearAxle[0,:]    , RearAxle[1,:], 'gray', LineWidth = 2.5)
		l5, = plt.plot(RightFrontTire[0,:] ,RightFrontTire[1,:], 'gray', LineWidth = 3)
		l6, = plt.plot(RightRearTire[0,:]  ,RightRearTire[1,:], 'gray', LineWidth = 3)
		l7, = plt.plot(LeftFrontTire[0,:]  ,LeftFrontTire[1,:], 'gray', LineWidth = 3)
		l8, = plt.plot(LeftRearTire[0,:]   ,LeftRearTire[1,:], 'gray', LineWidth = 3)


		for i in np.arange(0,len(x), self.interval):
			#calculate vehicle coordinates
			FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire = _plotVehicle(x[i], y[i], psi[i], delta[i], a, b, d, rW)
			xi = x[i]
			yi = y[i]

			l0.set_xdata(xi)
			l0.set_ydata(yi)
			l1.set_xdata(FrontBody[0,:])
			l1.set_ydata(FrontBody[1,:])
			l2.set_xdata(RearBody[0,:])
			l2.set_ydata(RearBody[1,:])
			l3.set_xdata(FrontAxle[0,:])
			l3.set_ydata(FrontAxle[1,:])
			l4.set_xdata(RearAxle[0,:])
			l4.set_ydata(RearAxle[1,:])
			l5.set_xdata(RightFrontTire[0,:])
			l5.set_ydata(RightFrontTire[1,:])
			l6.set_xdata(RightRearTire[0,:])
			l6.set_ydata(RightRearTire[1,:])
			l7.set_xdata(LeftFrontTire[0,:])
			l7.set_ydata(LeftFrontTire[1,:])
			l8.set_xdata(LeftRearTire[0,:])
			l8.set_ydata(LeftRearTire[1,:])

			plt.xlim([xi - self.windowSize, xi + self.windowSize])
			plt.ylim([yi - self.windowSize, yi + self.windowSize])

			self.f.canvas.draw()
			plt.pause(self.timeStep)

		plt.show()
		

class LocalState:
	def __init__(self, Ux=0.0, Uy=0.0, r=0.0, e=0.0, deltaPsi=0.0, s=0.0):
		self.Ux = Ux
		self.Uy = Uy
		self.r = r
		self.e = e
		self.deltaPsi = deltaPsi
		self.s = s

	def update(self, Ux = 0, Uy = 0, r = 0, e = 0, deltaPsi = 0, s = 0):
		self.Ux = Ux
		self.Uy = Uy
		self.r  = r
		self.e  = e
		self.deltaPsi = deltaPsi
		self.s = s

	def updateMapMatchStates(self, e, dPsi, s):
		self.e = e
		self.deltaPsi = dPsi
		self.s = s

	def updateVelocityState(self, Ux, Uy, r):
		self.Ux = Ux
		self.Uy = Uy
		self.r  = r

	def printState(self):
		print("Ux is %.2f" %self.Ux) 
		print("Uy is %.2f" %self.Uy)
		print("r is %.2f" %self.r)
		print("e is %.2f" %self.e)
		print("deltaPsi is %.2f" %self.deltaPsi)
		print("s is %.2f" %self.s)

		
class GlobalState:
	def __init__(self, path):

		#Start at 0 IC's if path is None
		if path is None: 
			self.posE = 0
			self.posN = 0
			self.psi = 0

		else:
			self.posE = path.posE[1] #start at second element of array to avoid mapMatch issues
			self.posN = path.posN[1]
			self.psi  = path.roadPsi[1]

	def update(self, posE = 0, posN = 0, psi = 0):
		self.posE = posE
		self.posN = posN
		self.psi  = psi

class Logger:
	#Probably a better way to decide this
	def __init__(self, NUMBER_DATA_POINTS = 100000):
		self.data = {}
		self.counter = 0
		self.NUMBER_DATA_POINTS = NUMBER_DATA_POINTS

	def append(self, signalName, signalData):
		if signalName in self.data.keys():
			self.data[signalName][self.counter] = signalData

		else:
			#create array once and append
			self.data[signalName] = np.zeros( (self.NUMBER_DATA_POINTS, 1) )
			self.data[signalName][0] = signalData

	def incrementCounter(self):
		self.counter = self.counter + 1


	def getData(self):
		#remove trailing zeros
		for key in self.data.keys():
			object = self.data[key]
			self.data[key] = self.data[key][0:self.counter-1, :]

		#Add additional info
		self.data["N"] = self.counter

		#return the dictionary
		return self.data


#Plays back recorded local state and allows second controller to simulate outputs on recorded data.
#Note - assumes controller is initialized to the same path and velocity profile as the recorded data.
class Replay:
	def __init__(self, logFile, controller):
		self.data = logFile
		self.controller = controller

	def simControllerOutputs(self):
		N = len(self.data["t"])
		inputs = {"deltaCmd": np.zeros((N, 1)), "FxCmd": np.zeros((N,1))}
		controlInput = ControlInput()
		localState = LocalState()

		print("Running through saved data")
		for i in range(N):
			Ux = self.data["Ux"][i]
			Uy = self.data["Uy"][i]
			r  = self.data["r" ][i]
			e  = self.data["e" ][i]
			deltaPsi = self.data["deltaPsi"][i]
			s  = self.data["s"][i]
			localState.update(Ux = Ux, Uy = Uy, r = r, e= e, deltaPsi = deltaPsi, s = s)

			self.controller.updateInput(localState, controlInput)
			inputs["deltaCmd"][i] = controlInput.delta
			inputs["FxCmd"][i] = controlInput.Fx

		return inputs



class MapMatch:
	def __init__(self, path, matchType):
		self.path = path
		self.seed = 1 #index of map to take a guess
		self.firstSearch = True #first time running search
		self.matchType = matchType
		if matchType == "embed":
			self.seed = 0
			self.REQUIRED_DISTANCE = 10.
			self.MAX_FORWARD_ITERS = 225
			self.MAX_BACKWARD_ITERS = 75



	def localize(self, localState, globalState):
		if self.matchType is "euler":
			return

		elif self.matchType is "closest":
			e, s, dPsi = self.mapMatch(globalState.posE, globalState.posN, globalState.psi)
			localState.updateMapMatchStates(e, dPsi, s)
			return

		elif self.matchType == "embed":
			e, s, dPsi = self.mapMatchEmbed(globalState.posE, globalState.posN, globalState.psi)
			localState.updateMapMatchStates(e, dPsi, s)
			return

		else:
			sys.exit("invalid mapMatch Type")

	def mapMatchEmbed(self, posE, posN, psi):
		e, s, K, psiDes, initStatus, numIters, smallestNorm = self.convertToLocalPathEMBED(posE, posN) 

		if initStatus == False:
			e = 0
			dPsi = 0
			s = 0

		else:

			#sanity check - s must be within map boundaries
			if s < 0:
				s = self.path.s[-1] + s #cycle to end of map
			elif s > self.path.s[-1]:
				s = s - self.path.s[-1]

			dPsi = psi - psiDes

			while dPsi > np.pi:
				dPsi = dPsi - 2 * np.pi

			while dPsi < -np.pi:
				dPsi = dPsi + 2 * np.pi


		return e, s, dPsi

	def mapMatch(self, posE, posN, psi):
		pEN = [posE, posN]
		pSE = self.convertToLocalPath(pEN)
		sEnd = self.path.s[-1] #last value of s

		#avoid small bug where we go to negative at start of path
		if pSE[0] < 0:
			s = sEnd + pSE[0]

		#wrap around if s > end of map
		elif pSE[0] > sEnd:
			s = pSE[0] - sEnd

		else:
			s = pSE[0]

		e = pSE[1]
		psiDes = np.interp(s, np.squeeze(self.path.s), np.squeeze(self.path.roadPsi))
		dPsi = psi - psiDes

		return e, s, dPsi

	def convertToLocalPathEmbed(self, posE, posN):
		#Very crude mapmatching -- works on small maps
		path = self.path

		m = len(self.path.s)
		EN = [posE, posN]

		#go forward

		lastPair = 9999999 #inf
		forwardInd = self.seed
		stillDecreasing = True
		numForwardIterations = 0

		while stillDecreasing and (numForwardIterations < self.MAX_FORWARD_ITERS):
			numForwardIterations = numForwardIterations + 1

			if forwardInd <= m - 2:
				currentPair = np.linalg.norm(EN - [path.posE[forwardInd]], path.posN[forwardInd])+np.linalg.norm(EN - [path.posE[forwardInd+1]], path.posN[forwardInd+1])
			else:
			#allow searching at the beginning of the map if world is closed 
				if path.isOpen:
					currentPair = 9999999
				else:
					currentPair = np.linalg.norm(EN - [path.posE[forwardInd]],
					path.posN[forwardInd]) + np.linalg.norm(EN - [path.posE[1]], path.posN[1])   

			stillDecreasing = currentPair < lastPair

			if stillDecreasing:
				lastPair = currentPair

				#allow searching at beginning of map if world is closed
				if (forwardInd == m-1) & (not path.isOpen):
					forwardInd = 1
				else:
					forwardInd += 1


		smallestF = lastPair

		#go backwards
		lastPair = 9999999 #inf
		backwardInd = self.seed
		stillDecreasing = True
		numBackwardIterations = 0

		while stillDecreasing & (numBackwardIterations < self.MAX_BACKWARD_ITERS):
			numBackwardIterations += 1

			if backwardInd >= 1:
				currentPair = np.linalg.norm(EN - [path.posE[backwardInd],
				path.posN[backwardInd]]) + np.linalg.norm(EN - [path.posE[backwardInd - 1],
				path.posN[backwardInd - 1]])

			else:
				#allow searching at end of map if map is closed
				if openWorld:
					currentPair = 9999999 #inf
				else:
					currentPair = np.linalg.norm(EN - [path.posE[backwardInd],
					path.posN[backwardInd]]) + np.linalg.norm(EN - [path.posE[m],
					path.posN[m]])


				stillDecreasing = currentPair < lastPair
				if stillDecreasing:
					lastPair = currentPair

				#allow searching from end of map if map is clsoed
				if (backwardInd ==0) & (not path.isOpen):
					backwardInd = m-1 
				else:
					backardInd = backwardInd - 1

		smallestB = lastPair

		if smallestB < smallestF:
			if backwardInd > 0:
				lowSind = backwardInd - 1

			else:
				lowSind = m - 2 
				#This should be m-1, but paths are defined so that the last
				#point overlaps with the first point. This will mess up the
				#cross product below, so we just go back one index when we cross
				#to the next lap

			highSind = backwardInd

		else:
			lowSind = forwardInd
			if forwardInd < m-1:
				highSind = forwardInd + 1
			else:
				highSind = 1 
				#This should be 0, but paths are defined so that the last point
				#overlaps with the first point. This messes up the cross product, 
				#so just go up one index when we cross to the next lap

		#need to track this for initialization testing
		smallestNorm = min(smallestB, smallestF)

		a = np.linalg.norm(EN-[path.posE[lowSind], path.posN[lowSind]])
		b = np.linalg.norm(EN-[path.posE[highSind], path.posN[highSind]])
		c = np.linalg.norm([path.posE[lowSind], path.posN[lowSind]]-[path.posE[highSInd], path.posN[highSInd]])

		deltaS = (a**2+c**2-b**2)/(2*c)
		abs_e = np.sqrt(np.abs(a**2 - deltaS**2))

		s = path.s[lowSind] + deltaS

		headingVector = [ -np.sin(path.roadPsi[lowSind]), np.cos(path.roadPsi[lowSind]), 0]
		pENaugmented = np.array([EN[0], EN[1], 0])    
		pathVector = [path.posE[lowSind], path.posN[lowSind] , 0]

		positionVector = pENaugmented -  pathVector
		crss = np.cross(headingVector, positionVector)

		e = np.sign(crss[2])*abs_e

		#compute K and psi desired via interpolation
		psiDes = path.roadPsi[lowSind] + (path.roadPsi[highSInd] - path.roadPsi[lowSind])/(path.s[highSind] - path.s[lowSind])*deltaS
		K =      path.curvature[lowSind]   + (path.curvature[highSInd] - path.curvature[lowSind])/(path.s[highSInd] - path.s[lowSind])*deltaS

		if smallestNorm < self.REQUIRED_DISTANCE:
			converged = True
			self.seed = lowSind

		else:
			converged = False
			self.seed = self.seed + self.MAX_BACKWARD_ITERS + self.MAX_FORWARD_ITERS

			#wrap around if necessary
			if self.seed > m-1:
				self.seed = 0

		iterations = numForwardIterations + numBackwardIterations

		return e, s, K, psiDes, initStatus, numIters, smallestNorm

	def convertToLocalPath(self, pEN):
		#reshape to rank 0 arrays
		posE = np.squeeze(self.path.posE)
		posN = np.squeeze(self.path.posN)
		roadPsi = np.squeeze(self.path.roadPsi)
		s = np.squeeze(self.path.s)


		m = posE.size  #number of points in the map

		if self.firstSearch is True:
			dist = np.zeros([m, 1]) #array of distances

			#go through all points in the map
			for i in range(m):
				pMap = [ posE[i], posN[i] ]
				dist[i] = np.linalg.norm(np.array(pEN) - np.array(pMap))


			# Get closest point and the corresponding distance
			absE = min(dist)
			idx = np.argmin(dist)

			#Use cross product to get the signed error
			#To determine sign of e, cross heading vector with vector from point to road
			#Append vectors with 0 to get 3 dims for convenient use of cross function
				
			#some weird stuff here to get both in same format for cross product    
			headingVector  = [-np.sin(roadPsi[idx]) , np.cos(roadPsi[idx]) , 0]
			pENaugmented = np.array([pEN[0], pEN[1], 0])    
			pathVector = [posE[idx], posN[idx] , 0]

			positionVector = pENaugmented -  pathVector
			crss = np.cross(headingVector, positionVector)

			pSE = np.zeros(2)
			pSE[0] = s[idx]    
			pSE[1] = np.sign(crss[2]) * absE

			self.firstSearch = False #next search use the seed
			self.seed = idx
			return pSE

		if self.firstSearch is False:
			#Go forward

			lastPair = 9999999.0 #Inf
			forwardInd = self.seed

			stillDecreasing = True

			while stillDecreasing:

				if forwardInd + 1 <= m-2:
					pMap1 = [ posE[forwardInd], posN[forwardInd] ]
					pMap2 = [ posE[forwardInd+1], posN[forwardInd+1] ]

					currentPair = np.linalg.norm( np.array(pEN) - np.array(pMap1) ) + np.linalg.norm( np.array(pEN) - np.array(pMap2)) 
				else:
					currentPair = 999999.0 #Inf

				stillDecreasing = currentPair < lastPair
				if stillDecreasing:
					lastPair = currentPair
					forwardInd = forwardInd + 1

			smallestForward = lastPair

			#Go back
			lastPair = 9999999.0 #inf
			backwardInd = self.seed
			stillDecreasing = True

			while stillDecreasing:
				if (backwardInd - 1) >= 1:
					pMap1 = [ posE[backwardInd], posN[backwardInd] ]
					pMap2 = [ posE[backwardInd -1], posN[backwardInd -1] ]

					currentPair = np.linalg.norm(np.array(pEN) - np.array(pMap1)) + np.linalg.norm(np.array(pEN) - np.array(pMap2))

				else:
					currentPair = 999999.0 #Inf

				stillDecreasing = currentPair < lastPair
				if stillDecreasing:
					lastPair = currentPair
					backwardInd = backwardInd - 1

			smallestBackward = lastPair

			if smallestBackward < smallestForward:
				lowSind = backwardInd - 1
				highSind = backwardInd

			else:
				lowSind = forwardInd
				highSind = forwardInd + 1

			#do not understand this math - need to think about further

			a = np.linalg.norm( np.array(pEN) - np.array([posE[lowSind] , posN[lowSind]]) )
			b = np.linalg.norm( np.array(pEN) - np.array([posE[highSind], posN[highSind]]))
			c = np.linalg.norm( np.array([posE[lowSind], posN[lowSind] ])- np.array([posE[highSind], posN[highSind]]) );
		 
			deltaS = (a**2+c**2-b**2)/(2*c)
			absE = np.sqrt(np.abs(a**2-deltaS**2))
		

			headingVector = [ -np.sin(roadPsi[lowSind]), np.cos(roadPsi[lowSind]), 0]
			pENaugmented = np.array([pEN[0], pEN[1], 0])
			pathVector = np.array([posE[lowSind], posN[lowSind], 0])

			positionVector = pENaugmented - pathVector
			crss = np.cross(headingVector, positionVector)
			
			pSE = np.zeros(2)
			pSE[0] = s[lowSind] + deltaS
			pSE[1] = np.sign(crss[2]) * absE

			self.seed = lowSind

			return pSE









#######################################################################################################################
####################################### HELPER FUNCTIONS ##############################################################
#######################################################################################################################

def  bicycleModel(vehicle, controlInput, localState, globalState, matchType, ts, K, weightTransferType, tireType):
	#Implementation of bicycle model with force derating, but no longitudinal dynamics

	#Unpack variables for brevity
	FxDes = controlInput.Fx
	delta = controlInput.delta
	

	Ux = localState.Ux    
	r = localState.r
	Uy = localState.Uy
	e = localState.e
	deltaPsi = localState.deltaPsi
	s = localState.s

	psi = globalState.psi
	posN = globalState.posN
	posE = globalState.posE

	m = vehicle.m
	a = vehicle.a
	b = vehicle.b
	Iz = vehicle.Iz

	#calculate forces and tire slips
	FxF, FxR = getFx(FxDes, Ux, vehicle)
	FzF, FzR = _getNormalForces(weightTransferType, FxF+FxR, vehicle)
	alphaF, alphaR = getSlips(localState, vehicle, controlInput)
	slips = {"alphaF": alphaF, "alphaR": alphaR}

	if tireType is "coupled":
		FyF, FyR, zetaF, zetaR = tm.coupledTireForces(alphaF, alphaR,  FxF, FxR, FzF, FzR, vehicle)
	elif tireType is "fiala":
		#just set FxF and FxR to 0.
		FyF, FyR, zetaF, zetaR = tm.coupledTireForces(alphaF, alphaR,  0., 0., FzF, FzR, vehicle)
	elif tireType is "linear":
		FyF = -vehicle.Cf * alphaF
		FyR = -vehicle.Cr * alphaR
		zetaF = 1.
		zetaR = 1.
	else:
		sys.exit("improper tire type specified")
	
	
	#Calculate state derivatives and update
	dUy = (FyF + FyR) / m - r*Ux
	dr  = (a*FyF - b*FyR) / Iz
	dUx = Uy * r + (FxF + FxR - FyF * delta) / m

	if matchType is "euler":
		de = Uy * np.cos(deltaPsi) + Ux * np.sin(deltaPsi)
		ds = Ux * np.cos(deltaPsi) - Uy * np.sin(deltaPsi)
		dDeltaPsi = r - K  * Ux

	dE = - Uy * np.cos(psi) - Ux * np.sin(psi)
	dN =   Ux * np.cos(psi) - Uy * np.sin(psi)
	dotPsi = r 

	Ax = dUx - r*Uy
	Ay = dUy + r*Ux 

	derivs = {"dUy": dUy, "dr": dr, "dUx": dUx, "dE": dE, "dN": dN, "dotPsi": dotPsi, "Ax": Ax, "Ay": Ay}

	#update states with Euler integration
	Uy = Uy + ts * dUy
	r  = r + ts * dr
	Ux = Ux + ts * dUx
	posE = posE + ts*dE
	posN = posN + ts*dN
	psi = psi + ts*dotPsi


	#For Euler integration, update states with ODEs 
	if matchType is "euler":
		e = e + ts*de 
		s = s + ts*ds
		deltaPsi = deltaPsi + ts * dDeltaPsi

	localState.update(Ux, Uy, r, e, deltaPsi, s)
	globalState.update(posE, posN, psi)

	forces = {"FyF": FyF, "FyR": FyR, "FxF": FxF, "FxR": FxR, "zetaF": zetaF, "zetaR": zetaR, "FzF": FzF, "FzR": FzR}
		
	return derivs, slips, forces 
	  

def getSlips(localState, veh, controlInput):
	Ux = localState.Ux
	Uy = localState.Uy
	r  = localState.r
	delta = controlInput.delta
	
	if Ux < 2.0:
		alphaF = 0 #speed too low to get slip estimate
		alphaR = 0

	else:
		alphaF = np.arctan( (Uy + veh.a * r) / Ux ) - delta
		alphaR = np.arctan( (Uy - veh.b * r) / Ux ) 

	return alphaF, alphaR


def getFx(FxDes, Ux, vehicle):

	#Implement engine and brake limits
	if FxDes > 0:
		if Ux == 0:
			Fx = FxDes #set to FxDes to avoid divide by zero
		else:
			Fx = min( vehicle.powerLimit / Ux - 0.7 * Ux ** 2 - 300, FxDes)
	else:
		Fx = FxDes

	#Distribute according to weight
	FxF = Fx * vehicle.b / vehicle.L
	FxR = Fx * vehicle.a / vehicle.L
	return FxF, FxR

def _plotVehicle(posE, posN, psi, delta, a, b, d, rW):
	#returns position of vehicle frame given coordinates of vehicle cg, steer angle, and dimensions a, b, d, and rW
	FrontAxle_Center_x = posE - a*np.sin(psi);
	FrontAxle_Center_y = posN + a*np.cos(psi);

	RearAxle_Center_x = posE + b*np.sin(psi);
	RearAxle_Center_y = posN - b*np.cos(psi);
	
	FrontAxle_Right_x = FrontAxle_Center_x + (d/2)*np.cos(psi);
	FrontAxle_Right_y = FrontAxle_Center_y + (d/2)*np.sin(psi);

	FrontAxle_Left_x = FrontAxle_Center_x - (d/2)*np.cos(psi);
	FrontAxle_Left_y = FrontAxle_Center_y - (d/2)*np.sin(psi);

	RearAxle_Right_x = RearAxle_Center_x + (d/2)*np.cos(psi);
	RearAxle_Right_y = RearAxle_Center_y + (d/2)*np.sin(psi);

	RearAxle_Left_x = RearAxle_Center_x - (d/2)*np.cos(psi);
	RearAxle_Left_y = RearAxle_Center_y - (d/2)*np.sin(psi);
	
	RightFrontTire_Front_x = FrontAxle_Right_x - rW*np.sin(psi+delta);
	RightFrontTire_Front_y = FrontAxle_Right_y + rW*np.cos(psi+delta);

	RightFrontTire_Back_x = FrontAxle_Right_x + rW*np.sin(psi+delta);
	RightFrontTire_Back_y = FrontAxle_Right_y - rW*np.cos(psi+delta);

	RightRearTire_Front_x = RearAxle_Right_x - rW*np.sin(psi);
	RightRearTire_Front_y = RearAxle_Right_y + rW*np.cos(psi);

	RightRearTire_Back_x = RearAxle_Right_x + rW*np.sin(psi);
	RightRearTire_Back_y = RearAxle_Right_y - rW*np.cos(psi);

	LeftFrontTire_Front_x = FrontAxle_Left_x - rW*np.sin(psi+delta);
	LeftFrontTire_Front_y = FrontAxle_Left_y + rW*np.cos(psi+delta);

	LeftFrontTire_Back_x = FrontAxle_Left_x + rW*np.sin(psi+delta);
	LeftFrontTire_Back_y = FrontAxle_Left_y - rW*np.cos(psi+delta);

	LeftRearTire_Front_x = RearAxle_Left_x - rW*np.sin(psi);
	LeftRearTire_Front_y = RearAxle_Left_y + rW*np.cos(psi);

	LeftRearTire_Back_x = RearAxle_Left_x + rW*np.sin(psi);
	LeftRearTire_Back_y = RearAxle_Left_y - rW*np.cos(psi);


	FrontBody =  np.array([[posE, FrontAxle_Center_x], [posN, FrontAxle_Center_y]]).squeeze()
	RearBody  =  np.array([[posE, RearAxle_Center_x] , [posN, RearAxle_Center_y]]).squeeze()
	FrontAxle =  np.array([[FrontAxle_Left_x, FrontAxle_Right_x], [FrontAxle_Left_y, FrontAxle_Right_y]]).squeeze()
	RearAxle  =  np.array([[RearAxle_Left_x, RearAxle_Right_x], [RearAxle_Left_y, RearAxle_Right_y]]).squeeze()
	RightFrontTire = np.array([[RightFrontTire_Front_x, RightFrontTire_Back_x],  [RightFrontTire_Front_y, RightFrontTire_Back_y]]).squeeze()
	RightRearTire  = np.array([[RightRearTire_Front_x, RightRearTire_Back_x],    [RightRearTire_Front_y, RightRearTire_Back_y]]).squeeze()
	LeftFrontTire  = np.array([[LeftFrontTire_Front_x, LeftFrontTire_Back_x],    [LeftFrontTire_Front_y, LeftFrontTire_Back_y]]).squeeze()
	LeftRearTire   = np.array([[LeftRearTire_Front_x, LeftRearTire_Back_x],      [LeftRearTire_Front_y, LeftRearTire_Back_y]]).squeeze()

	return FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire

def _getNormalForces(weightTransferType, Fx, veh):
	if weightTransferType is None:
		#just return the static normal forces
		FzF = veh.FzF
		FzR = veh.FzR

	if weightTransferType is "steadystate":
		L = veh.a + veh.b
		m = veh.m
		a = veh.a
		b = veh.b
		g = veh.g
		h = veh.h

		FzF = 1 / L * (m*b*g - h * Fx)
		FzR = 1 / L * (m*a*g + h * Fx)

	return FzF, FzR

