import numpy as np
import tiremodels as tm
import scipy
import pdb


#### General utility functions useful across libraries ######


#returns list of system matrices A, B
def getAllSys(veh, Ux, K, ts):
	A = []
	B = []
	D = []
	N = len(Ux)

	aF = np.zeros((N,1))
	aR = np.zeros((N,1))

	#Generate force lookup table
	numTableValues = 250
	
	#values where car is sliding
	alphaFslide = np.abs(np.arctan(3*veh.muF*veh.m*veh.b/veh.L*veh.g/veh.Cf)) 
	alphaRslide = np.abs(np.arctan(3*veh.muR*veh.m*veh.a/veh.L*veh.g/veh.Cr))

	alphaFtable = np.linspace(-alphaFslide, alphaFslide, numTableValues)
	alphaRtable = np.linspace(-alphaRslide, alphaRslide, numTableValues) # vector of rear alpha (rad)
	
	FyFtable = tm.fiala(veh.Cf, veh.muF, veh.muF, alphaFtable, veh.FzF)
	FyRtable = tm.fiala(veh.Cr, veh.muR, veh.muR, alphaRtable, veh.FzR)

	#flip arrays so Fy is increasing - important for numpy interp!!
	alphaFtable = np.flip(alphaFtable, 0)
	alphaRtable = np.flip(alphaRtable, 0)
	FyFtable = np.flip(FyFtable, 0) 
	FyRtable = np.flip(FyRtable, 0)

	for i in range(N):

		FyFdes = veh.b / veh.L * veh.m * Ux[i]**2 * K[i]
		FyRdes = veh.a / veh.L * veh.m * Ux[i]**2 * K[i]

		aF[i] = _force2alpha(FyFtable, alphaFtable, FyFdes)
		aR[i] = _force2alpha(FyRtable, alphaRtable , FyRdes)

		a, b, d = getAffineModel(Ux[i], K[i], ts[i], veh, aF[i], aR[i])
		
		A.append(a)
		B.append(b)
		D.append(d)



	return A, B, D


def getAffineModel(Ux, K, ts, veh, aFhat, aRhat):
	#unpack vehicle structure
	a = veh.a
	b = veh.b
	m = veh.m
	Cf = veh.Cf
	Cr = veh.Cr
	g = veh.g
	L = a + b
	Iz = veh.Iz
	FzF = veh.FzF
	FzR = veh.FzR
	muF = veh.muF
	muR = veh.muR


	FyFhat = tm.fiala(Cf, muF, muF, aFhat, FzF)
	FyRhat = tm.fiala(Cr, muR, muR, aRhat, FzR)
	Cf = getLocalStiffness(aFhat, Cf, muF, muF, FzF)
	Cr = getLocalStiffness(aRhat, Cr, muR, muR, FzR)

	#States: e, deltaPsi, r, beta
	Ac = np.array([ [0,  Ux,  0,  Ux] , [0, 0, 1, 0] , 
		[0, 0, (-a**2*Cf - b**2*Cr)/(Ux*Iz), (b*Cr - a*Cf)/Iz] , 
		[0,  0,(b*Cr - a*Cf)/(m*Ux**2)-1   , -(Cf+Cr)/(m*Ux)] ]) 


	Bc = [[0], [0], [a*Cf/Iz], [Cf/(m*Ux)]] 
	dc = [[0], 
		 [-K*Ux], 
		 [(a*Cf*aFhat - b*Cr*aRhat)/Iz + (a*FyFhat-b*FyRhat)/Iz ],
		 [(Cf*aFhat + Cr*aRhat)/(m*Ux) + (FyFhat + FyRhat)/(m*Ux)]]

	A, B1 = myc2d(Ac, np.concatenate((Bc, dc), axis = 1), ts)
	B = B1[:,0]
	#B = np.reshape(B, (B.size,1)) 
	d = B1[:,1]
	#d = np.reshape(d, (d.size,1))

	return A, B, d

def _force2alpha(forceTable, alphaTable, Fdes):
		if Fdes > max(forceTable):
			 Fdes = max(forceTable) - 1

		elif Fdes < min(forceTable):
			 Fdes = min(forceTable) + 1

		#note - need to slice to rank 0 for np to work
		#note - x values must be increasing in numpy interp!!!
		alpha = np.interp(Fdes, forceTable ,alphaTable)
		

		return alpha

def myc2d(A, B, ts):
	m, n = A.shape
	_, nb = B.shape

	conc1 = ts * np.concatenate((A,B), axis = 1)
	conc2 = np.zeros((nb, n+nb))
	conc  = np.concatenate( (conc1, conc2), axis = 0)

	s = scipy.linalg.expm(conc)
	Ad = s[0:n, 0:n]
	Bd = s[0:n, n:n+nb]

	return Ad, Bd
	

def getLocalStiffness(alpha, C, muP, muS, Fz):
	delta = 1e-4
	alpha2 = alpha + delta
	alpha1 = alpha - delta
	Fy1 = tm.fiala(C, muP, muS, alpha1, Fz)
	Fy2 = tm.fiala(C, muP, muS, alpha2, Fz)

	Cbar = (Fy2 - Fy1) / (alpha2 - alpha1)
	Cbar = abs(Cbar)

	return Cbar