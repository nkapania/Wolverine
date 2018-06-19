import numpy as np
import tiremodels as tm
import vehicle_lib
import path_lib
import math

class Controller:
	def __init__(self, path, vehicle, profile, controllerType = "lanekeeping"):
		self.path = path
		self.vehicle = vehicle
		self.profile = profile
		self.controllerType = controllerType
		self.parameters = {}
		if controllerType is "lanekeeping":
			self.parameters{'xLA'} = 14.2    #lookahead distance, meters
			self.parameters{'kLK'} = 0.0538   #proportional gain , rad / meter

	def calculateInput(self, localState):
		if controllerType is "lanekeeping":
			delta = lanekeeping(localState)
			Fx = speedTracking(localState)

		controlInput = ControlInput(delta,Fx)
		return controlInput


	def lanekeeping(self,localState):
		deltaFFW, betaFFW = getDeltaFFW(self, localState)
		deltaFB = getDeltaFB(self, localState, betaFFW)


	def getDeltaFB(self, localState, betaFFW):
		kLK = params['kLK']
		xLA = params['xLA']
		e = localState.e
		dPsi = localState.dPsi

		deltaFB = -kLK * (e + xLA * sin(dPsi + betaFFW))
		return deltaFB

	def getDeltaFFW(self, localState):
		a = self.vehicle.a
		b = self.vehicle.b
		L = self.vehicle.L
		m = self.vehicle.m

		Ux = localState.Ux
		K = localState.K


		FyFdes = b / L * m * Ux**2 * K
		FyRdes = a / b * FyFdes

		alphaFdes = force2alpha()







