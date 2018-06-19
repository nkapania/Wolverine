import matplotlib.pyplot as plt
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 

#Create vehicle object
shelley = vehicle_lib.Vehicle("nonlinear", "embed")

#Create path object
oval = path_lib.Path()
oval.loadFromCSV("maps/simpleRace.csv") #initialize by loading already defined map
oval.setFriction(0.7)

#Create speed profile
speedProfile = velocityprofile_lib.VelocityProfile("racing")
speedProfile.generate(oval, shelley)
