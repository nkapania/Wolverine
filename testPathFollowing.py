import matplotlib.pyplot as plt
from utils.vehicle_lib import *
from utils.tiremodel_lib import *
from utils.velocityprofile_lib import * 
from utils.path_lib import *
from utils.sim_lib import *
from utils.controllers import *
from numpy import genfromtxt


#Create vehicle object
shelley = Vehicle()

#Create path object
oval = Path()
oval.loadFromMAT("maps/simpleRace.mat")
print(oval.isOpen)

# Create speed profile
speedProfile = VelocityProfile("racing")
speedProfile.generate(shelley, oval, friction = 0.7, vMax = 50)

# #Create controller object - use lanekeeping
controller = LaneKeepingController(oval, shelley, speedProfile)

#simulate
bikeSim = Simulation(shelley, controller, path = oval, profile = speedProfile, mapMatchType = "closest") 
logFile = bikeSim.simulate()

#analyze results
bikeSim.plotResults()
