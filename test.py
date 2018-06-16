import matplotlib.pyplot as plt
import vehicles

veh = vehicles.vehicle("nonlinear", "embed")
print(veh.a)

plt.plot(veh.alphaFtable)
plt.plot(veh.alphaRtable)
plt.show()

