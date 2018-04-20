# Kalman filter example demo in Python

# A Python implementation of the example given in pages 11-15 of "An
# Introduction to the Kalman Filter" by Greg Welch and Gary Bishop,
# University of North Carolina at Chapel Hill, Department of Computer
# Science, TR 95-041,
# http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html

# by Andrew D. Straw

#reference for me youliang: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

import math
import numpy as np
import matplotlib.pyplot as plt

# plt.figure("measurement")
# plt.rcParams['figure.figsize'] = (10, 8)
# plt.title('Measurement w Noise', fontweight='bold')


# intial parameters
n_iter = 300
sz = (n_iter,) # size of array
x = -2.3 # truth value (typo in example at top of p. 13 calls this z)
delta_x = 0.05
x_truth = [x]
accel_const = 0
measurement_skip_rate = 15

#z = np.random.normal(x,1,size=sz) # observations (normal about x, sigma=0.1)

Q = 1e-5 # process variance

# allocate space for arrays
xhat=np.zeros(sz)      # a posteri estimate of x
P=np.zeros(sz)         # a posteri error estimate
xhatminus=np.zeros(sz) # a priori estimate of x
Pminus=np.zeros(sz)    # a priori error estimate
K=np.zeros(sz)         # gain or blending factor

z=np.zeros(sz)

vel_truth=np.zeros(sz)
vel_noise=np.zeros(sz)
accel_truth=np.zeros(sz)
accel_noise=np.zeros(sz)

R = 0.001**2 # estimate of measurement variance, change to see effect

# intial guesses
xhat[0] = 0.0
P[0] = 1.0

for k in range(1,n_iter):

    #measurement update
    if (k-1)%measurement_skip_rate == 0:   
        z[k] = np.random.normal(x,1.8,size=1)
    else:
        z[k] = z[k-1]

    #accel in odometry update
    if k > 1:
        vel_truth[k] = x_truth[k-1] - x_truth[k-2]
        if k > 2:
            accel_truth[k] =  vel_truth[k-1] - vel_truth[k-2]
            accel_noise[k] = np.random.normal(accel_truth[k],0.2,size=1)
            vel_noise[k] = accel_noise[k] + vel_noise[k-1]  # at + v'

    # time update
    xhatminus[k] = xhat[k-1] + vel_noise[k] + accel_noise[k] * 1/2 #equation 7
    Pminus[k] = P[k-1]+Q
    
    if (k-1)%measurement_skip_rate == 0:   
        # measurement update
        K[k] = Pminus[k]/( Pminus[k]+R )            #equation 19
        xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])    #equation 18
        P[k] = (1-K[k])*Pminus[k]
        print " = {} update displacement {}, z= {}".format(k, xhat[k], z[k])
    else:
        print "{}: accel {}, displacement: {}".format(k, accel_noise[k], xhatminus[k])
        xhat[k] = xhatminus[k]

    # x = x + delta_x
    x = x + math.sin(k/10) 
    x_truth.append(x)

# plt.plot(z, 'r+', label= 'measurement')


plt.figure("compare")
plt.plot(z, 'rx',label='noisy measurements')
plt.plot(xhat,'b-',label='a posteri estimate')
plt.plot(x_truth, 'b-',  color='g',label='truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('displacement')


# plt.figure()
# valid_iter = range(1,n_iter) # Pminus not valid at step 0
# plt.plot(valid_iter,Pminus[valid_iter],label='a priori error estimate')
# plt.title('Estimated $\it{\mathbf{a \ priori}}$ error vs. iteration step', fontweight='bold')
# plt.xlabel('Iteration')
# plt.ylabel('$(displacement)^2$')
# plt.setp(plt.gca(),'ylim',[0,.01])


plt.figure("Odometry ")
plt.subplot(2, 1, 1)
plt.plot(vel_truth, 'b.:', label = "truth")
plt.plot(vel_noise, 'r.', label = "noisy")
plt.title('velocity')
plt.ylabel('vel')
plt.legend()

plt.subplot(2, 1, 2)
plt.title('accel')
plt.plot(accel_truth, 'c.:', label = "truth")
plt.plot(accel_noise, 'r.', label = "noisy")

plt.xlabel('niter  (time, s)')
plt.ylabel('accel')
plt.legend()

plt.show()