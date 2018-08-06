import matplotlib.pyplot as plt
from utils.vehicle_lib import *
from utils.tiremodel_lib import *
from utils.velocityprofile_lib import * 
from utils.path_lib import *
from utils.sim_lib import *
from utils.controllers import *
from numpy import genfromtxt
import time
import pdb


#Create vehicle object
shelley = Vehicle()

#Create path object
oval = Path()
oval.loadFromMAT("maps/cpgOpen.mat")

#Create speed profile
speedProfile = VelocityProfile("racing")
speedProfile.generate(shelley, oval, friction = 0.7, vMax = 50)

# #Create controller object - use lanekeeping
controller = LaneKeepingController(oval, shelley, speedProfile)

#simulate
bikeSim = Simulation(shelley, controller, path = oval, profile = speedProfile, mapMatchType = "closest") 
logFile = bikeSim.simulate()


plt.close('all')

plt.figure()
plt.plot(logFile['s'], logFile['e'])
plt.xlabel('s (meters)')
plt.ylabel('e (meters)')


plt.figure()
plt.plot(logFile['s'], logFile['deltaPsi']*180 / np.pi)
plt.xlabel('s (meters)')
plt.ylabel('dPsi (deg)')

plt.figure()
plt.plot(logFile['s'], logFile['Ux'])
plt.plot(logFile['s'], logFile['UxDes'])
plt.xlabel('s (meters)')
plt.ylabel('Ux (m/s)')
plt.legend(['Actual', 'Desired'])
plt.show()