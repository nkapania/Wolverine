import matplotlib.pyplot as plt
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 
import sim_lib
import controllers
import numpy as np
from numpy import genfromtxt

#Test harness to check that PySim tracks MATLAB simulation



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
FxF  = x[:,22]
FxR  = x[:,23]
Fx = FxF + FxR
FyF = x[:,24]
FyR = x[:,25]
posE = x[:,26]
posN = x[:,27]
psi = x[:,28]

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
FxF_sim = np.zeros ( UxDesired.shape )
FxR_sim = np.zeros ( UxDesired.shape ) 
alphaF_sim = np.zeros ( UxDesired.shape )
alphaR_sim = np.zeros ( UxDesired.shape )
FyF_sim = np.zeros ( UxDesired.shape )
FyR_sim = np.zeros ( UxDesired.shape )
s_sim = np.zeros ( UxDesired.shape )
e_sim = np.zeros ( UxDesired.shape )
dPsi_sim = np.zeros ( UxDesired.shape )

globalState = sim_lib.GlobalState(oval)
localState = sim_lib.LocalState()
controlInput = controllers.ControlInput()

mapM = sim_lib.MapMatch(oval, "closest")

m = s.size

for i in range(m):
	globalState.update( posE[i], posN[i], psi[i])
	mapM.localize(localState, globalState)
	s_sim[i] = localState.s
	e_sim[i] = localState.e


plt.close("all")
plt.figure()
plt.plot(e_sim)
plt.plot(e)
plt.legend(['PySim', 'MATLAB'])

plt.show()





