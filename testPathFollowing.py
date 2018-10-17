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
track.loadFromMAT("maps/thunderhill_race.mat")
#track.generateRandomWorld(numTurns = 10)

# Create speed profile
speedProfile = RacingProfile(veh, track, friction = 0.90, vMax = 99)

# #Create controller object - use lanekeeping
#controller = NeuralNetFeedforward(track, veh, speedProfile)
controller = LaneKeepingController(track, veh, speedProfile)

#simulate
bikeSim = Simulation(veh, controller, path = track, profile = speedProfile, 
	mapMatchType = "closest", tires = "coupled", maxTime = 200.) 
logFile = bikeSim.simulate()

#analyze results
bikeSim.plotResults(xaxis = "t")

# # #animate car
#anim = MyAnimation(logFile, track, veh, timeStep = bikeSim.ts, interval = 5)
#anim.run()


#write to .mat file
#bikeSim.save('logs/mu9')