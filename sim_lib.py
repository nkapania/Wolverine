import numpy as np
import tiremodel_lib as tm
import vehicle_lib
import path_lib
import controllers
import sys

#Defines a simulation class and a state class

class Simulation:
	def __init__(self, path, vehicle, profile, controller): 
		self.path = path
		self.vehicle = vehicle
		self.profile = profile
		self.controller = controller
		self.isRunning = True
		self.physics = "bicycle"
		self.logger = Logger()
		self.ts = 0.01
		self.mapMatchType = "euler"
		
		
	def simulate(self):
		#initialize states
		Ux0 = self.profile.Ux[0] #start car at the correct velocity
		localState = LocalState(Ux0)
		globalState = GlobalState(self.path)
		controlInput = controllers.ControlInput()
		counter = 0
		log = Logger()
		

		while self.isRunning:

			#Perform localization
			localState = mapMatch(localState, globalState, self.path, self.mapMatchType)	

			#Check to see if we should terminate
			self.checkForTermination(localState, counter)

			#Calculate controller inputs
			controlInput, auxVars = self.controller.updateInput(localState, controlInput)

			#Update state
			localState, globalState = self.updateState(controlInput, localState, globalState, auxVars)

			#Append counter and print to screen
			counter = counter + 1
			printStatus(localState, self.path, counter)
			#print(localState.s)
			#print(self.path.s[-1])



			#save signals needed
			K, UxDes = auxVars	#unpack auxillary variables

			log.append('t',counter*self.ts)
			log.append('Ux',localState.Ux)
			log.append('s', localState.s)
			log.append('e', localState.e)
			log.append('UxDes', UxDes) 
			log.append('posE', globalState.posE)
			log.append('posN', globalState.posN)
			log.incrementCounter()



		return log.getData()





	def checkForTermination(self, localState, counter):

		#Check if we have ended the simulation
		if localState.s > (self.path.s[-1] - 0.55): #Stop simulation a little before end of path
			self.isRunning = False
			runTime = counter * sim.ts
			print("Simulation complete - total time %02d sec" %runTime) 

		#Check if we have gone off the track	
		if abs(localState.e) > 5.0:
			print("Car has left the track - terminating...")
			self.isRunning = False

		#For debug
		if counter is 1000:
			self.isRunning = False
			print("Simulation Terminating")


	def updateState(self, controlInput, localState, globalState, auxVars):
		(K, UxDes) = auxVars #unpack auxvars
		if self.physics is "bicycle":
			localState, globalState = bicycleModel(self.vehicle, controlInput, localState, globalState, self.mapMatchType, self.ts, K)
			return localState, globalState

class LocalState:
	def __init__(self, Ux=0.0, Uy=0.0, r=0.0, e=0.0, deltaPsi=0.0, s=0.0, K = 0.0):
		self.Ux = Ux
		self.Uy = Uy
		self.r = r
		self.e = e
		self.deltaPsi = deltaPsi
		self.s = s

	def update(self, Ux, Uy, r, e, deltaPsi, s):
		self.Ux = Ux
		self.Uy = Uy
		self.r  = r
		self.e  = e
		self.deltaPsi = deltaPsi
		self.s = s

		
class GlobalState:
	def __init__(self, path):
		self.posE = path.posE[1] #start at second element of array to avoid mapMatch issues
		self.posN = path.posN[1]
		self.psi  = path.roadPsi[1]

	def update(self, posE, posN, psi):
		self.posE = posE
		self.posN = posN
		self.psi  = psi

class Logger:
	def __init__(self, NUMBER_DATA_POINTS = 10000):
		self.data = {}
		self.counter = 0
		self.NUMBER_DATA_POINTS = NUMBER_DATA_POINTS

	def append(self, signalName, signalData):
		if signalName in self.data.keys():
			self.data[signalName][self.counter] = signalData

		else:
			#create array once
			self.data[signalName] = np.zeros( (self.NUMBER_DATA_POINTS, 1) )

	def incrementCounter(self):
		self.counter = self.counter + 1


	def getData(self):
		#remove trailing zeros
		for key in self.data.keys():
			object = self.data[key]
			self.data[key] = np.trim_zeros(object, 'b')

		#return the dictionary
		return self.data





def  bicycleModel(vehicle, controlInput, localState, globalState, matchType, ts, K):
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
	alphaF, alphaR = getSlips(localState, vehicle, controlInput)
	FyF, FyR = tm.coupledTireForces(alphaF, alphaR,  FxF, FxR, vehicle)

    
	#Calculate state derivatives
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

    #update states
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
        
	return localState, globalState  
      



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
			Fx = min( vehicle.powerLimit / Ux , FxDes)
	else:
		Fx = FxDes

	#Distribute according to weight
	FxF = Fx * vehicle.b / vehicle.L
	FxR = Fx * vehicle.a / vehicle.L
	return FxF, FxR


#localization to frenet frame   
def mapMatch(localState, globalState, path, matchType):
		if matchType is "euler":
			return localState

		#to be implemented - embedded map matching
		elif matchType is "embed":
			sys.exit("Error - embedded not implemented yet")

def printStatus(localState, path, counter):
	pctComplete = np.ceil( 100 * localState.s / path.s[-1] )
	if np.mod(counter, 100) == 0:
		print("Simulation is %02d percent done" % pctComplete)
		#print(pctComplete)
		#print("Distance Along Path is %04d meters") % localState.s