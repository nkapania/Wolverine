import numpy as np
#import tiremodels

#currently just uses Shelley values
class vehicle:
	def __init__(self, tireType, mapMatchType): 
		
		self.mapMatchType = mapMatchType
		self.tireType = tireType
		self.a = 1.0441 #CG to front wheelbase [m]
		self.b = 1.4248 #CG to rear wheelbase [m] 
		self.m = 1512.4 #vehicle mass (kg)
		self.Cf = 160000.0 #vehicle cornering stiffness (N)
		self.Cr = 180000.0 #vehicle cornering stiffness (N)
		self.Iz  = 2.25E3  #vehicle inertia (kg  m^2)
		self.xLA = 14.2    #lookahead distance, meters
		self.kLK = .0538   #proportional gain , rad / meter
		self.muF = .97     #front friction coeff
		self.muR = 1.02    #rear friction coeff
		self.g = 9.81      #m/s^2, accel due to gravity
		self.L = self.a + self.b #total vehicle length, m
		self.FzF = self.m*self.b*self.g/self.L   #Maximum force on front vehicles
		self.FzR = self.m*self.a*self.g/self.L   #Maximium force on rear vehicles
		self.D = 0.3638 #Drag coefficient
		self.h = 0.75   #Distance from the ground
		self.alphaFlim = 7.0 * np.pi / 180 #rad
		self.alphaRlim = 5.0 * np.pi / 180 #rad 
		self.alphaFslide = np.abs(np.arctan(3*self.muF*self.m*self.b/self.L*self.g/self.Cf)) 
		self.alphaRslide = np.abs( np.arctan(3*self.muR*self.m*self.a/self.L*self.g/self.Cr))
		self.brakeTimeDelay = 0.25 #Seconds
		self.rollResistance = 255.0 #Newtons
		self.Kx = 3000.0; #Speed tracking gain
		self.powerLimit = 16000.0 #Watts
		self.numTableValues = 250
		self.alphaFtable = np.linspace(-self.alphaFslide,self.alphaFslide,self.numTableValues)
		self.alphaRtable = np.linspace(-self.alphaRslide,self.alphaRslide,self.numTableValues) # vector of rear alpha (rad)
		
		self.alphaRtable = self.alphaRtable.reshape(self.numTableValues,1)
		self.alphaFtable = self.alphaFtable.reshape(self.numTableValues,1)

		# # if tireType is "linear":
		# # 	veh.FyFtable = -veh.Cf*veh.alphaFrontTable
		# # 	veh.FyRtable = -veh.Cr*veh.alphaRearTable
			
		# # elseif tireType is "nonlinear":
		# # 	veh.FyFtable = tireforces(self.Cf, self.muF, self.muF, self.alphaFrontTable, self.FzF)
		# # 	veh.FyRtable = tireforces(self.Cr, self.muR, self.muR, self.alphaRearTable,  self.FzR)




