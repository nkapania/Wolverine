import matplotlib.pyplot as plt
from utils.vehicles import *
from utils.paths import *
from utils.pathgeneration import *
import scipy.io as sio
import numpy as np 

veh = Vehicle(vehicleName = "shelley")

# #Create path object
track = Path()
track.loadFromMAT("maps/THcenter.mat")

# #load in road bounds
bounds = sio.loadmat('maps/thunderhill_bounds_shifted.mat')

rpg = RapidPathGeneration(veh, track, bounds, mu = 0.9, NUM_ITERS = 1)
plt.plot(rpg.widthLeft)
plt.plot(rpg.widthRight)
plt.show()

results = rpg.optimize()

print(results.lapTimes)


