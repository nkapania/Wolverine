import matplotlib.pyplot as plt
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 
import sim_lib
import controllers
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


#simulate
bikeSim = sim_lib.Simulation(oval, shelley, speedProfile, controller)
logFile = bikeSim.simulate()



#check against MATLAB simulation results
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


plt.close("all")
plt.figure()
plt.plot(logFile['s'], logFile['e'])
plt.plot(s, e)
plt.xlabel('Distance Along Path (m)')
plt.ylabel('Lateral Error (m)')
plt.legend(['PySim', 'MATLAB'])

plt.figure()
plt.plot(logFile['s'], logFile['UxDes'])
plt.plot(s, UxDesired)
plt.xlabel('Time (s)')
plt.ylabel('Distance Along Path (m)')
plt.legend(['PySim', 'MATLAB'])

plt.figure()
plt.plot(logFile['s'], logFile['r'])
plt.plot(s, r)
plt.xlabel('Distance Along Path (s)')
plt.ylabel('Lateral Error (m)')
plt.legend(['PySim', 'MATLAB'])

plt.show()

