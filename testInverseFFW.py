import matplotlib.pyplot as plt
from utils.vehicles import *
from utils.velocityprofiles import *
from utils.tiremodels import *
from utils.simulation import *
from utils.paths import *
from utils.control import *

#Create vehicle object
veh = Vehicle(vehicleName = "shelley")

#Create path object
track = Path()
track.loadFromMAT("maps/THrace.mat")

# Create speed profile
speedProfile = BasicProfile(veh, track, friction = 0.95, vMax = 99)

# #Create controller object - use lanekeeping
controllerNN = LKwithNN_feedforward(track, veh, speedProfile)
controllerLK = LaneKeepingController(track, veh, speedProfile)

#simulate
bikeSimNN = Simulation(veh, controllerNN, track, speedProfile, mapMatchType = "closest", maxTime = 20) 
logFile1 = bikeSimNN.simulate()

#analyze results
bikeSimNN.plotResults()
#bikeSimLK.plotResults()

# #animate car
#anim = MyAnimation(logFile, track, veh, timeStep = bikeSim.ts, interval = 5)
#anim.run()

#write to .mat file
#bikeSim.save('logs/test2')