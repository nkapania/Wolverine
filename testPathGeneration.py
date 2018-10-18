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

rpg = RapidPathGeneration(veh, track, bounds, mu = 0.9, NUM_ITERS = 5)
results = rpg.optimize()

vps = results.vps
opt = results.opt
logs = results.logFiles
paths = results.path
lapTimes = results.lapTimes


opt_e = opt[1]["e"] 
opt_left = opt[1]["widthLeft"]
opt_right = opt[1]["widthRight"]
optPosE = opt[1]["posE"]
optPosN = opt[1]["posN"]
optS = opt[1]["s"]
optK = opt[1]["K"]

plt.figure()
plt.plot(opt_e, label = "opt")
plt.plot(opt_left, label = "left")
plt.plot(opt_right, label = "right")
plt.legend()

refPath = paths[0]
optPath = paths[1]

plt.figure()
plt.plot(optPosE, optPosN,'bo', label = "raw opt")
plt.plot(refPath["posE"], refPath["posN"], label = "ref path")
plt.plot(optPath["posE"], optPath["posN"], label = "processed opt")
plt.plot(bounds["in"][:,0], bounds["in"][:,1],'k', label = "bounds")
plt.plot(bounds["out"][:,0], bounds["out"][:,1],'k', label = "bounds")
plt.axis("equal")
plt.legend()

plt.figure()
plt.plot(refPath["s"], refPath["K"], label = "ref path")
plt.plot(optS, optK, label = "raw opt")
plt.plot(optPath["s"], optPath["K"], label = "refined opt")
plt.legend()

plt.show()











