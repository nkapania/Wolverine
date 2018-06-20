import numpy as np
import tiremodels as tm
import vehicle_lib
import path_lib
import math


#Controller from Nitin Kapania's PhD thesis - lookahead with augmented sideslip for
#steering feedback, longitudinal is simple PID feedback control
class LaneKeepingController():
	def __init__(self, path, vehicle):
		self.path = path
		self.vehicle = vehicle
		self.xLA = 14.2    #lookahead distance, meters
		self.kLK = 0.0538  #proportional gain , rad / meter
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


	def getDeltaFB(self, localState, betaFFW):
		kLK = params['kLK']
		xLA = params['xLA']
		e = localState.e
		dPsi = localState.dPsi

		deltaFB = -kLK * (e + xLA * sin(dPsi + betaFFW))
		return deltaFB

	def speedTracking(localState):
		Fx = Fxff = m*AxDes + sign(Ux)*fdrag*Ux^2 + frr*sign(Ux); % Feedforward
    	Fxfb = -kp_fx*(Ux - UxDes); % Feedback
    	FxCommand = Fxff + Fxfb; 
    	return 

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

		









