#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 18 11:02:54 2018

@author: nkapania
"""
import scipy.io as sio
import matplotlib as plt
from Learn_Model import *
from Learning_Functions import *
from Learning_Params import *



#%%

trainCostBike, trainCostNN, trainCostRNN =  LearnModel(1, 2)
trainCostBike1, trainCostNN1, trainCostRNN1 =  LearnModel(1, 2, 'nitin')


#%%
plt.figure()
plt.plot(trainCostNN,   'b', label="NN")
plt.plot(trainCostRNN,  'r', label="RNN")
plt.plot(trainCostBike, 'g', label="Bike")
plt.xlabel("Training Epoch")
plt.ylabel("MSE of Velocity State Predictions")

