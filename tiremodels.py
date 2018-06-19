#Library of tire models
import numpy as np

#Fiala model. Calculates nonlinear tire curve using Fiala brush model with no longitudinal force.  Inputs are the cornering stiffness per axle (N/rad). The muP and muS are slip and slip friction coefficients. alpha = slip angle (rad) Fz = normal load on that axle (N)
def fiala(C, muP, muS, alpha, Fz):
	alphaSlide = np.abs(np.arctan( 3*muP*Fz / C ))
	Fy = np.zeros(alpha.size)

	for i in range(alpha.size):

		#Use 3rd order polynomial equation when below the tire range
		 if np.abs(alpha[i]) < alphaSlide:
		 	Fy[i] = -C * np.tan(alpha[i]) + C**2 / (3 * muP * Fz) * (2 - muS / muP) * np.tan(alpha[i]) * np.abs(np.tan(alpha[i])) - C**3/(9*muP**2*Fz**2)*np.tan(alpha[i])**3*(1-(2*muS)/(3*muP)) 
		 else :
		#Use sliding force otherwise
		 	Fy[i] = -muS * Fz * np.sign(alpha[i])

	return Fy




		
		
