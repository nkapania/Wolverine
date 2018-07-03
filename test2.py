import matplotlib.pyplot as plt
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 
import sim_lib
import controllers
import numpy as np
from numpy import genfromtxt


#Create vehicle object
shelley = vehicle_lib.Vehicle()

#Create path object
oval = path_lib.Path()
oval.loadFromCSV("maps/simpleRace.csv") #initialize by loading already defined map
oval.setFriction(0.7)

#Create speed profile
speedProfile = velocityprofile_lib.VelocityProfile("racing")
speedProfile.generate(shelley, oval)

#Create controller object - use lanekeeping
controller = controllers.LaneKeepingController(oval, shelley, speedProfile)


#load MATLAB inputs and outputs


#simulate
bikeSim = sim_lib.Simulation(oval, shelley, speedProfile, controller)



x = genfromtxt("unitTest.csv", delimiter =",")

delta = x[:,0]
deltaFFW = x[:,1]
deltaFB = x[:,2]
curvature = x[:,3]
FxDes = x[:,4]
UxDesired = x[:,5]
t = x[:,6]
alphaR = x[:,7]
alphaF = x[:,8]
s = x[:,9]
Ux = x[:,10]
Uy = x[:,11]
r  = x[:,12]
e  = x[:,13]
deltaPsi = x[:,14]
betaFFW = x[:,15]
FyFdes = x[:,16]
FyRdes = x[:,17]
alphaFdes = x[:,18]
alphaRdes = x[:,19]
FxFFW = x[:,20]
FxST = x[:,21]

delta_sim = np.zeros( delta.shape )
deltaFFW_sim = np.zeros( deltaFFW.shape )
deltaFB_sim  = np.zeros( deltaFB.shape )
curvature_sim = np.zeros ( curvature.shape )
betaFFW_sim = np.zeros ( betaFFW.shape )
FyFdes_sim = np.zeros ( FyFdes.shape )
FyRdes_sim = np.zeros ( FyRdes.shape )
alphaFdes_sim = np.zeros ( alphaFdes.shape)
alphaRdes_sim = np.zeros ( alphaRdes.shape)
alpha_sim = np.zeros( alphaRdes.shape )
FxDes_sim = np.zeros( FxDes.shape )
UxDes_sim = np.zeros( UxDesired.shape )
FxFFW_sim = np.zeros( UxDesired.shape )
FxFB_sim = np.zeros( UxDesired.shape )


localState = sim_lib.LocalState()


for i in range( delta.size ):
	localState.update(Ux[i], Uy[i], r[i], e[i], deltaPsi[i], s[i])
	FxDes_sim[i], UxDes_sim[i], FxFFW_sim[i], FxFB_sim[i] = controllers._speedTracking(controller, localState)




plt.plot(FxDes)	
plt.plot(FxDes_sim, linestyle = '--')
plt.legend(['MATLAB','PySim'])
plt.show()

