import matplotlib.pyplot as plt
from utils.vehicle_lib import *
from utils.tiremodel_lib import *
from utils.velocityprofile_lib import * 
from utils.path_lib import *
from utils.sim_lib import *
from utils.controllers import *
from numpy import genfromtxt


#Create vehicle object
shelley = Vehicle(vehicleName = "shelley")

#Create path object
track = Path()
track.loadFromMAT("maps/THrace.mat")

# Create speed profile
speedProfile = VelocityProfile("racing")
speedProfile.generate(shelley, track, friction = 0.9, vMax = 99)

# #Create controller object - use lanekeeping
controller = LaneKeepingController(track, shelley, speedProfile)

#simulate
bikeSim = Simulation(shelley, controller, path = track, profile = speedProfile, mapMatchType = "closest", weightTransferType = None, tires = "linear") 
logFile = bikeSim.simulate()

#analyze results
#bikeSim.plotResults()

#animate car
# anim = MyAnimation(logFile, track, shelley, timeStep = bikeSim.ts, interval = 5)
# anim.run()

#write to .mat file
#bikeSim.save('logs/test1.mat')

plt.plot(logFile["s"], logFile["FyF"])
plt.plot(logFile["s"], logFile["FyR"])
plt.show()