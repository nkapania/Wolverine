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
#track.generateRandomWorld(numTurns = 10)

# Create speed profile
speedProfile = BasicProfile(veh, track, friction = 0.9, vMax = 99)

# #Create controller object - use lanekeeping
#controller = NeuralNetFeedforward(track, veh, speedProfile)
controller = LaneKeepingController(track, veh, speedProfile)

#simulate
bikeSim = Simulation(veh, controller, path = track, profile = speedProfile, mapMatchType = "closest") 
logFile = bikeSim.simulate()

#analyze results
#bikeSim.plotResults()

# # #animate car
#anim = MyAnimation(logFile, track, veh, timeStep = bikeSim.ts, interval = 5)
#anim.run()


#write to .mat file
bikeSim.save('logs/mu9')