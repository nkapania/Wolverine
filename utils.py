import numpy as np
import tiremodels as tm



#### General utility functions useful across libraries ######


#returns list of system matrices A, B
def getAllSys(veh, Ux, K, ts):
	A = []
	B = []
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

		aF[i] = _force2alpha(FyFtable, alphaFrontTable, FyFdes)
		aR[i] = _force2alpha(FyRtable, alphaRearTable , FyRdes)

		a, b, d = getAffineModel(Ux[i], K[i], ts[i], veh, aF[i], aR[i])
		
		A.append[a]
		B.append[b]
		D.append[d]


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


	FyFhat = tm.fiala(Cf, muF, muF, afHat, FzF)
	FyRhat = tm.fiala(Cr, muR, muR, arHat, FzR)
	Cf = getLocalStiffness(afHat, Cf, muF, muF, FzF)
	Cr = getLocalStiffness(arHat, Cr, muR, muR, FzR)

	#States: e, deltaPsi, r, beta
	Ac = np.array([ [0,  Ux,  0,  Ux] , [0, 0, 1, 0] , 
		[0, 0, (-a**2*Cf - b**2*Cr)/(Ux*Iz), (b*Cr - a*Cf)/Iz] , 
        [0,  0,(b*Cr - a*Cf)/(m*Ux**2)-1   , -(Cf+Cr)/(m*Ux)] ]) 


	Bc = [[0], [0], [a*Cf/Iz], [Cf/(m*Ux)]]; 
	
	##CONTINUE HERE ########

	#dc = [[0], [-K*Ux]; (a*Cf*afHat - b*Cr*arHat)/Iz + (a*FyFhat-b*FyRhat)/Iz; (Cf*afHat + Cr*arHat)/(m*Ux) + (FyFhat + FyRhat)/(m*Ux)];

	#[A B1] = myc2d(Ac, [Bc dc], ts);
	#B = B1(:,1);
	#d = B1(:,2);







def _force2alpha(forceTable, alphaTable, Fdes):
        if Fdes > max(forceTable):
             Fdes = max(forceTable) - 1

        elif Fdes < min(forceTable):
             Fdes = min(forceTable) + 1

        #note - need to slice to rank 0 for np to work
        #note - x values must be increasing in numpy interp!!!
        alpha = np.interp(Fdes, forceTable ,alphaTable)
        

        return alpha
