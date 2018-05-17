import numpy as np
import matplotlib.pyplot as plt

# plt.axis([0, 10, 0, 1])

# for i in range(10):
#     y = np.random.random()
#     # plt.scatter(i, y)
#     print "hihi"
#     plt.plot(i, y, 'go-', label = "truth")
#     plt.pause(0.05)

# plt.show()


x=0
y=0 

fig=plt.figure(1)
ax=fig.add_subplot(111)
ax.set_xlim(0,10)
ax.set_ylim(0,10)
line,=ax.plot(x,y,'ko-')

for i in range(10):
    x = np.concatenate((line.get_xdata(),[i]))

    y = np.concatenate((line.get_ydata(),[i]))

    print x, y
    line.set_data(x,y)
    plt.pause(1)