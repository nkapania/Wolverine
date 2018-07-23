#Nathan Spielberg
#7.2.18

import numpy as np

#Two dictionaries (one for veh param one for other params)
Veh           = {}
Param         = {} 

#Define Constants
Param["EPOCHS"]          = 1000 #200
Param["LEARNING_RATE"]   = 0.001 #Adam LR #50.0 #GD LR
Param["TRAIN_PERCENT"]   = 1.0 #1.0 #.9 #Dev on the other 30 percent
Param["BATCH_SIZE"]      = 1000
Param["N1"]              = 500 #Neurons first layer
Param["N2"]              = 250 #Neurons second layer
Param["PLOT"]            = True
Param["DT"]              = 0.01 #was .01
Param["FS"]              = (1.0/Param["DT"])*2.0*np.pi
Param["CUTOFF"]          = 6.0*2.0*np.pi #Really need to check on this.... 0.25
#IE we dont care about anything above 6 hz

Param["N_SAMPLES"]       = 1000 #Number of state transition trajectories.
Param["NN_MODEL"]        = True
Param["PLOT"]            = True
Param["N_STATE_INPUT"]   = 4
Param["INSPECT_TIRES"]   = False
Param["ADD_NOISE"]       = True
#Param["VEHICLE_NAME"]  = "x1" 
Param["VEHICLE_NAME"]    = "Niki"

#Training Options
Param["RESTORE"]         = False  #Set to true to restore model in existing file path
Param["SAVE"]            = False
Param["TRAIN"]           = True #Set to true is we want to train a model.

Param["eps"]             = 10e-6
Param["RNN"]             = True   #Use the RNN network Model Try one hidden state at first....
Param["TWO_FRIC"]        = False
Param["GEN_DATA"]        = True
Param["CONST_SPEED"]     = False
Param["Weight_Transfer"] = False
#TRAIN_FILE    = "mattdata/t08_18mps.mat"
TRAIN_FILE               = 'Niki/07g_2017-08-18_ae.mat' #Deal right now with about 100k samples
#TRAIN_FILE    = 'Niki/rampsteer_left_2ndgear_2018-04-10_aa.mat' #Deal right now with about 100k samples
#TRAIN_FILE    = 'Niki/rampsteer_right_2ndgear_2018-04-10_aa.mat' #Deal right now with about 100k samples
#TRAIN_FILE    = 'Niki/test_3_model_learning_2018-06-27_aa.mat'

#bc loading into ram not off hd- need to work more on input system

Param["T"]               = 4       #Number of delay states in the RNN Model 
Param["sim_time"]        = 1.0     #Simulate for 1 second so far...



#Single State note good enough!

#Change data generation tomorrow and then look at rnns?

#Now Ux is an unput and is random in the initial range!
Param["Ux_lim"]        = 15.0 #20

#For initial parameter guesses for bike model- mean and std dev for gaussian sampling around 1.0
Param["loc"]           = 1.1
Param["scale"]         = 0.1

if Param["VEHICLE_NAME"]  == "x1":
    Veh["a"]       = 1.53 #1.194   #1.1669 #1.2408 Trudy #Niki param updated as of 6/24/18
    Veh["b"]       = 1.23 #1.437   #1.4631#1.6062 Trudy
    Veh["l"]       = Veh["a"] + Veh["b"] 
    Veh["m"]       = 2000 #1776.2  #1745.0 #was 1902 Trudys
    Veh["h"]       = 0.55
    Veh["Izz"]     = 3.7638e3 #2763.49 #3824.0 #2763
    Veh["Cf"]      = 160.0e3 #145k
    Veh["Cr"]      = 170.0e3 #151k
    Veh["mu"]      = 0.9
    Veh["mu_2"]    = 0.3   #Second friction value for identification
    Veh["del_lim"] = 27.0*np.pi/180.0
    Veh["p_lim"]   = 100*147*1e3 #Niki Engine Power Limit
    Veh["b_bias"]  = .66         # 66 percent of braking force to the front of the car
else:
    Veh["a"]       = 1.194   #1.1669 #1.2408 Trudy #Niki param updated as of 6/24/18
    Veh["b"]       = 1.437   #1.4631#1.6062 Trudy
    Veh["l"]       = Veh["a"] + Veh["b"] 
    Veh["h"]       = 0.55
    Veh["m"]       = 1776.2  #1745.0 #was 1902 Trudys
    Veh["Izz"]     = 2763.49 #3824.0 #2763
    Veh["Cf"]      = 150.0e3 #145k
    Veh["Cr"]      = 170.0e3 #151k
    Veh["mu"]      = 0.9
    Veh["mu_2"]    = 0.3   #Second friction value for identification
    Veh["del_lim"] = 27.0*np.pi/180.0
    Veh["p_lim"]   = 100*147*1e3 #Niki Engine Power Limit
    Veh["b_bias"]  = .66         # 66 percent of braking force to the front of the car
