import matplotlib.pyplot as plt
from utils.vehicles import *
from utils.paths import *
from utils.pathgeneration import *
import scipy.io as sio


veh = Vehicle(vehicleName = "shelley")

#Create path object
track = Path()
track.loadFromMAT("maps/thunderhill_race.mat")

#load in road bounds
bounds = sio.loadmat('maps/thunderhill_bounds_shifted.mat')

vp = RacingProfile(veh, track, 0.9)


rpg = RapidPathGeneration(veh, track, bounds)
path, vp = rpg.optimize()


