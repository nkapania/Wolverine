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
track.loadFromMAT("maps/cpgSmooth.mat")

track.isOpen = 1

# Create speed profile
speedProfile1 = BasicProfile(shelley, track, friction = 0.5, vMax = 30)
speedProfile2 = BasicProfile(shelley, track, friction = 0.5, vMax = 30, AxMax = 2.0)


plt.figure()
plt.subplot(2, 1, 1)
plt.plot(speedProfile1.s, speedProfile1.Ux)
plt.plot(speedProfile2.s, speedProfile2.Ux)

plt.subplot(2, 1, 2)
plt.plot(speedProfile1.s, speedProfile1.Ax)
plt.plot(speedProfile2.s, speedProfile2.Ax)
plt.show()


#
## #Create controller object - use lanekeeping
#controller = LaneKeepingController(track, shelley, speedProfile)
#
#
##simulate 2 times and save to mat file
#bikeSim1 = Simulation(shelley, controller, path = track, profile = speedProfile, 
#	mapMatchType = "closest", weightTransferType = None, tires = "fiala", maxTime = 15.) 
#
#l1 = bikeSim1.simulate()
#bikeSim1.plotResults()
#bikeSim1.save('logs/test1')
#
#
#bikeSim2 = Simulation(shelley, controller, path = track, profile = speedProfile, 
#	mapMatchType = "closest", weightTransferType = None, tires = "coupled", maxTime = 20.) 
#
#l2 = bikeSim2.simulate()
#bikeSim2.save('logs/test2')

