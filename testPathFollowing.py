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
oval.loadFromMAT("maps/rightTurnRFSdownshifted.mat")
oval.setFriction(0.3)


#Create speed profile
speedProfile = VelocityProfile("racing")
speedProfile.generate(shelley, oval)

# #Create controller object - use lanekeeping
controller = LaneKeepingController(oval, shelley, speedProfile)

#simulate
bikeSim = Simulation(shelley, controller, path = oval, profile = speedProfile, mapMatchType = "closest") 
logFile = bikeSim.simulate()



plt.figure()
plt.plot(logFile['s'], logFile['UxDes'])
plt.plot(logFile['s'], logFile['Ux'])
#plt.plot(s, UxDesired)
plt.xlabel('s (meters)')
plt.ylabel('Desired Velocity (m)')
plt.legend(['Desired', 'Actual'])


plt.figure()
plt.plot(logFile['s'], logFile['e'])
#plt.plot(s, UxDesired)
plt.xlabel('s (meters)')
plt.ylabel('e (meters)')
#plt.legend(['PySim', 'MATLAB'])
plt.show()



