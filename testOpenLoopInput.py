import matplotlib.pyplot as plt
from vehicle_lib import *
from tiremodel_lib import *
from velocityprofile_lib import * 
from path_lib import *
from sim_lib import *
from controllers import *
from numpy import genfromtxt
import time


#Create vehicle object
shelley = Vehicle()

#Create controller object 
controller = OpenLoopControl(shelley, delta = 2*np.pi / 180, Fx = 100.)

#simulate for 100 seconds
bikeSim = Simulation(shelley, controller, maxTime = 0.1) 
logFile = bikeSim.simulate()

print(logFile['deltaCmd'])



#plt.figure()
#plt.plot(logFile['s'], logFile['UxDes'])
#plt.plot(logFile['s'], logFile['Ux'])
##plt.plot(s, UxDesired)
#plt.xlabel('s (meters)')
#plt.ylabel('Desired Velocity (m)')
#plt.legend(['PySim', 'MATLAB'])
#
#
plt.figure()
plt.plot(logFile['s'], logFile['Uy'])
##plt.plot(s, UxDesired)
#plt.xlabel('s (meters)')
#plt.ylabel('e (meters)')
##plt.legend(['PySim', 'MATLAB'])
#plt.show()
#


