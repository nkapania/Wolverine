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
track.loadFromMAT('maps/THrace.mat')
#track.generateRandomWorld(numTurns = 10)

plt.figure()
plt.plot(track.s, track.curvature)
plt.show()
