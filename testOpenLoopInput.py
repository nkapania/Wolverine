import matplotlib.pyplot as plt
from utils.vehicle_lib import *
from utils.tiremodel_lib import *
from utils.velocityprofile_lib import * 
from utils.path_lib import *
from utils.sim_lib import *
from utils.controllers import *
import time


#Create vehicle object
shelley = Vehicle()

#Create controller object 
controller = OpenLoopControl(shelley, delta = 3.8*np.pi / 180, Fx = 400.)

#simulate for 100 seconds
bikeSim = Simulation(shelley, controller, maxTime = 2., vStart = 20.) 
logFile = bikeSim.simulate()

plt.subplot(3,1,1)
plt.plot(logFile['t'], logFile['Uy'])
plt.xlabel('t (sec)')
plt.xlim(0,2)
plt.ylabel('Uy (m/s)')

plt.subplot(3,1,2)
plt.plot(logFile['t'], logFile['r'])
plt.xlabel('t (sec)')
plt.ylabel('r (rad/s')
plt.xlim(0,2)


plt.subplot(3,1,3)
plt.plot(logFile['t'], logFile['Ux'])
plt.xlabel('t (sec)')
plt.ylabel('Ux (meters/s')
plt.ylim(15, 25)
plt.xlim(0,2)


plt.show()
