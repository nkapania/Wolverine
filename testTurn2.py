import matplotlib.pyplot as plt
from utils.vehicles import *
from utils.velocityprofiles import *
from utils.tiremodels import *
from utils.simulation import *
from utils.paths import *
from utils.control import *
import scipy.io as sio

#Create vehicle object
shelley = Vehicle(vehicleName = "shelley")

#Create path object
track = Path()
track.loadFromMAT('maps/thunderhill_race.mat')
#track.generateRandomWorld(numTurns = 10)


# # #bp = BasicProfile (shelley, track, friction = 0.9, vMax = 99)
rp = RacingProfile(shelley, track, friction = 0.9, vMax = 99)
# Fv2, G, Mv2, MvDot, theta = rp.makePath3D()
#rp.findSpeedProfile(Fv2, G, Mv2, MvDot, theta)

plt.plot(rp.s, rp.Ux)
plt.show()

#out = {"s": rp.s, "Fv2": Fv2, "G": G, "Mv2": Mv2, "MvDot": MvDot, "theta": theta, "UxDesAlg": rp.UxDesiredAlgebraic}
out = {"s": rp.s, "Ux": rp.Ux, "Ax": rp.Ax}
sio.savemat("out1",out)

