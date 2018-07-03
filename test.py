import matplotlib.pyplot as plt
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 
import sim_lib
import controllers

#Create vehicle object
shelley = vehicle_lib.Vehicle()

#Create path object
oval = path_lib.Path()
oval.loadFromCSV("maps/simpleRace.csv") #initialize by loading already defined map
oval.setFriction(0.7)

#Create speed profile
speedProfile = velocityprofile_lib.VelocityProfile("racing")
speedProfile.generate(shelley, oval)

#Create controller object - use lanekeeping
controller = controllers.LaneKeepingController(oval, shelley, speedProfile)


#simulate
bikeSim = sim_lib.Simulation(oval, shelley, speedProfile, controller)
logFile = bikeSim.simulate()


# plt.plot(oval.posE, oval.posN)
# plt.axis("equal")
# plt.plot(logFile['posE'], logFile['posN'])

plt.plot(logFile['s'], logFile['e'])
plt.show()

