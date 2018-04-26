import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Data for plotting

size = 200
z1 = np.random.normal(0.2,0.1,size=200)
z2 = np.random.normal(0,0.1,size=200)


plt.figure("IMU RAW READING ")
plt.subplot(3, 1, 1)
plt.title('X axis accel vs iteration (with offset)')
plt.plot(z1, 'r.:')
plt.ylabel('accel (ms^-2)')

plt.subplot(3, 1, 2)
plt.title('X axis accel vs iteration (without offset)')
plt.plot(z2, 'c.:')
plt.ylabel('accel (ms^-2)')

plt.show()