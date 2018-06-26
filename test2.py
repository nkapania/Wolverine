import matplotlib.pyplot as plt
import vehicle_lib 
import tiremodel_lib 
import velocityprofile_lib 
import path_lib 
import sim_lib
import controllers
import numpy as np
import time

#Create vehicle object
shelley = vehicle_lib.Vehicle()

#Create path object
oval = path_lib.Path()
oval.loadFromCSV("maps/simpleRace.csv") #initialize by loading already defined map




