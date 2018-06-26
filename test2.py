import matplotlib.pyplot as plt
import sim_lib
import numpy as np

log = sim_lib.Logger(500)

b = 0.0
for i in range(0,100):

	if i < 50: 
		a = 0
		b = 0
	else:
		a = np.random.randint(0,10)
		b = b + 0.01

	
	log.append('a',a)
	log.append('b',b)
	log.incrementCounter()



logFile = log.getData()

aP = logFile['a']
bP = logFile['b']

plt.plot(bP, aP)
plt.show()


print( aP.shape )
print( bP.shape )