import matplotlib.pyplot 
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 

shelley = veh.vehicle("nonlinear", "embed")
vp = velocityProfile(path, shelley, 0.9, 99, .95)
