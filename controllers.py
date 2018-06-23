import numpy as np
import tiremodels as tm
import vehicle_lib
import path_lib
import math


#Controller from Nitin Kapania's PhD thesis - lookahead with augmented sideslip for
#steering feedback, longitudinal is simple PID feedback control
class LaneKeepingController():
	def __init__(self, path, vehicle, profile):
		self.path = path
		self.vehicle = vehicle
		self.profile = profile
		self.xLA = 14.2    #lookahead distance, meters
		self.kLK = 0.0538  #proportional gain , rad / meter
		self.kSpeed = 1000.0 #Speed proportional gain - N / (m/s)
		self.alphaFlim = 7.0 * np.pi / 180 #steering limits for feedforward controller
		self.alphaRlim = 5.0 * np.pi / 180 #steering limits for feedforward controller

		#Initialize force lookup tables for feedforward
		numTableValues = 250

		#values where car is sliding
		alphaFslide = np.abs(np.arctan(3*self.muF*self.m*self.b/self.L*self.g/self.Cf)) 
		alphaRslide = np.abs(np.arctan(3*self.muR*self.m*self.a/self.L*self.g/self.Cr))

		alphaFtable = np.linspace(-alphaFslide, alphaFslide, numTableValues)
		alphaRtable = np.linspace(-alphaRslide, alphaRslide, numTableValues) # vector of rear alpha (rad)
		
		self.alphaFtable = alphaFtable.reshape(numTableValues,1)
		self.alphaRtable = alphaRtable.reshape(numTableValues,1)

		self.FyFtable = tm.fiala(vehicle.Cf, vehicle.muF, vehicle.muF, self.alphaFtable, vehicle.FzF)
		self.FyRtable = tm.fiala(vehicle.Cr, vehicle.muR, vehicle.muR, self.alphaRtable, vehicle.FzR)


	def calculateInput(self, localState):
			delta = lanekeeping(localState)
			Fx = speedTracking(localState)

		controlInput = ControlInput(delta,Fx)
		return controlInput


	def lanekeeping(self,localState):
		deltaFFW, betaFFW = getDeltaFFW(self, localState)
		deltaFB = getDeltaFB(self, localState, betaFFW)
		delta = deltaFFW + deltaFB
		return delta, deltaFFW, deltaFB


	def getDeltaFB(self, localState, betaFFW):
		kLK = self.kLK
		xLA = self.xLA
		e = localState.e
		dPsi = localState.dPsi

		deltaFB = -kLK * (e + xLA * sin(dPsi + betaFFW))
		return deltaFB

	def speedTracking(self, localState):
		AxTable = self.profile.AxDes
		UxTable = self.profile.Ux
		sTable = self.profile.s
		s = localState.s
		Ux = localState.Ux

		AxDes = np.interp(sTable, s, AxTable) #run interp every time - this is slow, but we may be able to get away with
		UxDes = np.interp(sTable, s, UxTable) 

		alpha = np.interp(forceTable, Fdes, alphaTable)


		FxFFW = m*AxDes #+ np.sign(Ux)*fdrag*Ux^2 + frr*sign(Ux); % Feedforward
    	FxFB = -self.kSpeed*(Ux - UxDes); % Feedback
    	FxCommand = FxFFW + FxFB
    	return Fx

	def getDeltaFFW(self, localState):
		a = self.vehicle.a
		b = self.vehicle.b
		L = self.vehicle.L
		m = self.vehicle.m

		Ux = localState.Ux
		K = localState.K


		FyFdes = b / L * m * Ux**2 * K
		FyRdes = a / b * FyFdes

		alphaFdes = force2alpha(self.FyFtable, self.alphaFrontTable, FyFdes)
		alphaRdes = force2alpha(self.FyRtable, self.alphaRearTable , FyRdes)

		betaFFW = alphaRdes + b * K 
		deltaFFW = K * L + alphaRdes - alphaFdes
		return deltaFFW, betaFFW


	def force2alpha(forceTable, alphaTable, Fdes):
		if Fdes > np.max(forceTable):
			Fdes = max(forceTable) - 1

		elif Fdes < np.min(forceTable):
			Fdes = min(forceTable) + 1

		alpha = np.interp(forceTable, Fdes, alphaTable)

		return alpha

		









