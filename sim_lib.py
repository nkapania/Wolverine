import numpy as np
import tiremodels as tm
import vehicle_lib
import path_lib
import controllers

#Defines a simulation class and a state class


class Simulation:
	def __init__(self, path, vehicle, profile, controller): 
		self.path = path
		self.vehicle = vehicle
		self.profile = profile
		self.localizationType = localizationType
		self.controller = controller
		self.isRunning = True
		self.physics = "bicycle"
		self.logger = Logger()
		self.ts = 0.01
		self.mapMatchType = "euler"
		
	def simulate(self):
		#initialize states
		localState = LocalState(Ux = profile.Ux[0])
		globalState = GlobalState(self.path)
		controlInput = ControlInput()
		counter = 0
		

		while self.isRunning:

			#Perform localization
			localState, matchObject = mapMatch(localState, globalState, self.path, self.mapMatchType)

			#Check to see if we should terminate
			self.checkForTermination()

			#Calculate controller inputs
			controlInput = controller.calculateInput(localState, matchObject)

			#Update state
			localState, globalState = updateState(controlInput)

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


	def updateState(self, controlInput):
		if physics is "bicycle":
			localState, globalState = bicycleModel(self.vehicle, controlInput, localState)




		



def bicycleModel(vehicle, controlInput, localState):
	#Implementation of bicycle model with force derating, but no longitudinal dynamics
	FxDes = controlInput.Fx
	Ux = localState.Ux	

	FxF, FxR = getFx(FxDes, Ux, vehicle)
	
    FyF, FyR = tm.coupledTireForces(alphaF, alphaR,  FxF, FxR, vehicle)
      


	       #  dUy = (FyF + FyR)/veh.m - r*Ux;
        # dr = (veh.a*FyF - veh.b*FyR)/veh.Iz;
        # dUx = Uy*r + (FxF + FxR - FyF*delta)/veh.m;
        # if strcmp(veh.mapMatch, 'euler')
        #     de = Uy*cos(dPsi) + Ux*sin(dPsi);
        #     ds = Ux*cos(dPsi) - Uy*sin(dPsi);
        #     ddPsi = r - K*Ux;
        # end

        # dE = -Uy*cos(psi)-Ux*sin(psi);
        # dN = Ux*cos(psi) - Uy*sin(psi);
        # dotPsi = r;



def getFx(FxDes, Ux, vehicle):

	#Implement engine and brake limits
	if FxDes > 0:
		Fx = np.min( vehicle.powerLimit / Ux , FxDes)
	else:
		Fx = FxDes

	#Distribute according to weight
	FxF = Fx * vehicle.b / vehicle.L
	FxR = Fx * vehicle.a / vehicle.L
	return FxF, FxR




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
		
class GlobalState:
	def __init__(path):
		self.posE = path.posE[1] #start at second element of array to avoid mapMatch issues
		self.posN = path.posN[1]
		self.psi  = path.roadPsi[1]

class ControlInput:
	def __init__(self):
		self.Fx = 0.0
		self.delta = 0.0





# class MatchObject:
# 	def __init__(self):
# 		self.idx = 1
# 		self.numIters = 0
# 		self.interpFraction = 0.0


#class Logger:
#	def __init__(self):