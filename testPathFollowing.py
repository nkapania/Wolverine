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
oval.loadFromMAT("maps/rightTurnRFSdownshifted.mat")
oval.setFriction(0.5)


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



