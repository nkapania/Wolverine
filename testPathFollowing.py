import matplotlib.pyplot as plt
from utils.vehicles import *
from utils.velocityprofiles import *
from utils.tiremodels import *
from utils.simulation import *
from utils.paths import *
from utils.control import *

#Create vehicle object
shelley = Vehicle(vehicleName = "shelley")

#Create path object
track = Path()
track.loadFromMAT("maps/THrace.mat")

# Create speed profile
speedProfile = VelocityProfile("racing")
speedProfile.generate(shelley, track, friction = 1.0, vMax = 99)

# #Create controller object - use lanekeeping
controller = LaneKeepingController(track, shelley, speedProfile)

#simulate
bikeSim = Simulation(shelley, controller, path = track, profile = speedProfile, mapMatchType = "closest", weightTransferType = None, tires = "fiala", maxTime = 20.) 
logFile = bikeSim.simulate()

#analyze results
#bikeSim.plotResults()

# #animate car
# anim = MyAnimation(logFile, track, shelley, timeStep = bikeSim.ts, interval = 5)
# anim.run()


#write to .mat file
bikeSim.save('logs/test2')