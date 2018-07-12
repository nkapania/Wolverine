import matplotlib.pyplot as plt
from vehicle_lib import *
from tiremodel_lib import *
from velocityprofile_lib import * 
from path_lib import *
from sim_lib import *
from controllers import *
from numpy import genfromtxt
import time

#Create vehicle object
shelley = Vehicle()

#Create path object
oval = Path()
oval.loadFromMAT("maps/THrace.mat")
oval.setFriction(0.7)


#Create speed profile
speedProfile = VelocityProfile("racing")
speedProfile.generate(shelley, oval)

# #Create controller object - use lanekeeping
controller = LaneKeepingController(oval, shelley, speedProfile)

#simulate
bikeSim = Simulation(oval, shelley, speedProfile, controller, "closest")
logFile = bikeSim.simulate()





# #check against MATLAB simulation results
# x = genfromtxt("unitTest.csv", delimiter =",")

# delta = x[:,0]
# deltaFFW = x[:,1]
# deltaFB = x[:,2]
# curvature = x[:,3]
# FxDes = x[:,4]
# UxDesired = x[:,5]
# t = x[:,6]
# alphaR = x[:,7]
# alphaF = x[:,8]
# s = x[:,9]
# Ux = x[:,10]
# Uy = x[:,11]
# r  = x[:,12]
# e  = x[:,13]
# deltaPsi = x[:,14]
# betaFFW = x[:,15]
# FyFdes = x[:,16]
# FyRdes = x[:,17]
# alphaFdes = x[:,18]
# alphaRdes = x[:,19]
# FxFFW = x[:,20]
# FxST = x[:,21]
# FxF  = x[:,22]
# FxR  = x[:,23]
# Fx = FxF + FxR
# FyF = x[:,24]
# FyR = x[:,25]



plt.figure()
plt.plot(logFile['s'], logFile['UxDes'])
plt.plot(logFile['s'], logFile['Ux'])
#plt.plot(s, UxDesired)
plt.xlabel('Time (s)')
plt.ylabel('Distance Along Path (m)')
plt.legend(['PySim', 'MATLAB'])
plt.show()

# # plt.figure()
# # plt.plot(logFile['s'], logFile['r'])
# # plt.plot(s, r)
# # plt.xlabel('Distance Along Path (m)')
# # plt.ylabel('Lateral Error (m)')
# # plt.legend(['PySim', 'MATLAB'])

# # plt.figure()
# # plt.plot(logFile['s'], logFile['Uy'])
# # plt.plot(s, Uy)
# # plt.xlabel('Distance Along Path (m)')
# # plt.ylabel('Lateral Velocity (m / s)')
# # plt.legend(['PySim', 'MATLAB'])

# # plt.figure()
# # plt.plot(logFile['posE'], logFile['posN'])
# # plt.plot(oval.posE, oval.posN)
# # plt.xlabel('East (m)')
# # plt.ylabel('North (m)')
# # plt.legend(['Actual', 'Desired'])
# # plt.grid(True)
# # plt.axis("equal")

# # plt.figure()
# # plt.plot(logFile['s'], logFile['Ux'])
# # plt.plot(s, Ux)
# # plt.xlabel('Distance Along Path (m)')
# # plt.ylabel('Vehicle Speed (m)')
# # plt.legend(['PySim', 'MATLAB'])
# # plt.grid(True)
# # plt.axis("equal")

# # plt.show()
