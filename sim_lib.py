import numpy as np
import tiremodels as tm
import vehicle_lib
import path_lib
import controllers

#Defines a simulation class and a state class


class Simulation:
	def __init__(self, path, vehicle, profile, localizationType = "euler", controllerType = "lanekeeping"): 
		self.path = path
		self.vehicle = vehicle
		self.profile = profile
		self.localizationType = localizationType
		self.controllerType = controllerType
		self.isRunning = True
		self.physics = "bicycle"
		self.logger = Logger()
		self.ts = 0.01
		
	def simulate(self):
		#initialize states
		localState = LocalState(Ux = profile.Ux[0])
		globalState = GlobalState(self.path)
		controlInput = ControlInput()
		counter = 0
		controller = Controller(self.path, self.vehicle, self.profile, self.controllerType)


		while self.isRunning:

			#Perform localization
			localState = mapMatch(localState, globalState, self.path, self.mapMatchType)

			#Check to see if we should terminate
			self.checkForTermination()

			#Calculate controller inputs
			controlInput = controller.calculateInput(localState)

			#Update state
			localState, globalState, intermediates = updateState(controlInput)

			#Update logger
			Logger.append(localState, globalState, controlInput, intermediates) 

			#Append counter and print to screen
			printStatus(localState, path)

			counter = counter + 1


	def checkForTermination(self):

		#Check if we have ended the simulation
		if self.localState.s > self.path.s[-1] - 0.55: #Stop simulation a little before end of path
			self.isRunning = false

		#Check if we have gone off the track	
		if abs(self.localState.e) > 5.0:
			print("Car has left the track - terminating...")
			self.isRunning = false
		

def mapMatch(self, localState, globalState, path, matchType):
		if matchType is "euler":
			return
		elif matchType is "embed":
			#to be implemented - embedded map matching


class LocalState:
	def __init__(Ux=0.0, Uy=0.0, r=0.0, e=0.0, dPsi=0.0, s=0.0, K = 0.0):
		self.Ux = Ux
		self.Uy = Uy
		self.r = r
		self.e = e
		self.dPsi = dPsi
		self.s = s
		self.K = 0.0

class GlobalState:
	def __init__(path):
		self.posE = path.posE[1]
		self.posN = path.posN[1]
		self.psi  = path.roadPsi[1]

class ControlInput:
	def __init__(self):
		self.Fx = 0.0
		self.delta = 0.0


class Logger:
	def __init__(self):