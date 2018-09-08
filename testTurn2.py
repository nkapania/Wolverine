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
track.loadFromMAT('maps/thunderhill_race.mat')
#track.generateRandomWorld(numTurns = 10)

rp = RacingProfile(shelley, track, friction = 0.7, vMax = 99)

plt.plot(rp.s, rp.Ux)
plt.show()