import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import signal
import threading
import numpy as np

x_list = [[1,2], [2,3], [10, 8], [4.3, 6], [2.1, 8.6]]
x_list2= [[1,2,3,4,5,6,7],[1,2,3,4,3,2,1]]

plt.figure("Encoder Odometry")
# plt.subplot(4, 1, 1)
# plt.title('odom XY - t')
# plt.plot(odomX_list, 'b.:')
# plt.ylabel('meter')
# plt.plot(odomY_list, 'm.:')

plt.title('odom XY - t')

plt.subplot(2, 1, 1)
plt.title('odom XY - t')
plt.plot(x_list, 'b.:')
plt.ylabel('meter')


plt.subplot(2, 1, 2)
plt.plot(x_list2, 'b.:')
plt.plot([1,2,3,4,5,6,7], [1,2,3,4,3,2,1], 'ro')
plt.axis([0, 10, 0, 10])
plt.show()