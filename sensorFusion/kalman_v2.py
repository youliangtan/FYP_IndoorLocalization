#reference for me youliang: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

import math
import numpy as np
import matplotlib.pyplot as plt

# plt.figure("measurement")
# plt.rcParams['figure.figsize'] = (10, 8)
# plt.title('Measurement w Noise', fontweight='bold')


# intial parameters
n_iter = 800
sz = (n_iter,) # size of array
x = -2.3 # truth value (typo in example at top of p. 13 calls this z)
v = 0
delta_x = 0.05
x_truth = [x]
accel_const = 0
measurement_skip_rate = 20
accel_reduction_factor = 0.5

#z = np.random.normal(x,1,size=sz) # observations (normal about x, sigma=0.1)

Q = 5e-3 # process variance
Q = np.array([[Q, 0], [0, Q]])

# allocate space for arrays
# xhat=np.zeros(sz)      # a posteri estimate of x
# P=np.zeros(sz)         # a posteri error estimate
# xhatminus=np.zeros(sz) # a priori estimate of x
# Pminus=np.zeros(sz)    # a priori error estimate
# K=np.zeros(sz)         # gain or blending factor

K= np.zeros(shape= (n_iter, 2, 2))
xhat=np.zeros(shape= (n_iter, 2, 1))      # a posteri estimate of []
P=np.zeros(shape= (n_iter, 2, 2))         # a posteri error estimate
xhatminus=np.zeros( shape= (n_iter, 2, 1) ) # a priori estimate of x
Pminus=np.zeros( shape= (n_iter, 2, 2) )    # a priori error estimate

#z=np.zeros(sz)

z=np.zeros(shape= (n_iter, 2, 1))
z[0][0] = x
vel_truth=np.zeros(sz)
vel_noise=np.zeros(sz)
accel_truth=np.zeros(sz)
accel_noise=np.zeros(sz)

R = 0.01**2 # estimate of measurement variance, change to see effect
R = np.array([[R, 0], [0, R ]])

# intial guesses
xhat[0] = 0.0
P[0] = 1.0

print xhat[0], P[0]

def firstEle(x):
    return x[0]


def secondEle(x):
    return x[1]


def plotGraph():
    # plt.plot(z, 'r+', label= 'measurement')
    
    plt.figure("compare")
    plt.plot( map(firstEle, z), 'rx',label='noisy measurements')
    plt.plot( map(firstEle, xhat),'b-',label='a posteri estimate')
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
    plt.plot(map(secondEle, xhat), 'c.', label = "kalman")
    plt.title('velocity')
    plt.ylabel('vel')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.title('Accel')
    plt.plot(accel_truth, 'c.:', label = "truth")
    plt.plot(accel_noise, 'r.', label = "noisy")

    plt.xlabel('niter  (time, s)')
    plt.ylabel('accel')
    plt.legend()

    plt.show()



def main():
    global xhat, xhatminus, P, Pminus, K, z, x, x_truth
    global vel_noise, vel_truth, accel_noise, accel_truth
    
    A_matrix = np.array([[1, 1], [0,  1]])
    B_matrix = np.array([[0.5], [1]])

    for k in range(1,n_iter):

        # ====================== measurement update (vision) ==============================
        if (k-1)%measurement_skip_rate == 0:   
            z[k][0] = np.random.normal(x, 1.8, size=1)
            
            ##sudden noise from vision
            if k == 901:
                z[k][0] = 200

            #update speed
            z[k][1] = (z[k][0] - z[k-1][0])/measurement_skip_rate #divide by time 
        else:
            z[k][0] = z[k-1][0]

        # ==================== accel in odometry update ========================
        if k > 1:
            vel_truth[k] = x_truth[k-1] - x_truth[k-2]
            if k > 2:
                accel_truth[k] =  vel_truth[k-1] - vel_truth[k-2]
                accel_noise[k] = np.random.normal(accel_truth[k],0.2,size=1) * accel_reduction_factor
                vel_noise[k] = accel_noise[k] + vel_noise[k-1]  # at + v'

        # =================time update (imu dependency) ====================
        #xhatminus[k] = xhat[k-1] + vel_noise[k] + accel_noise[k] * 1/2 #equation 7
        xhatminus[k] = np.matmul( A_matrix, xhat[k-1] ) + accel_noise[k] * B_matrix #equation 7
        # Pminus[k] = P[k-1] + Q
        Pminus[k] = np.matmul( np.matmul(A_matrix, P[k-1]) , np.linalg.inv( A_matrix)) + Q    # P = A*P*A^T + Q
        
        # ================= measurement update ===========================
        if (k-1)%measurement_skip_rate == 0:   
            print "{}: xhatminus {} {}".format(k, xhatminus[k][0], xhatminus[k][1])
          
            # print Pminus[k], "THIS\n\n" , ( Pminus[k]+R )
            K[k] = np.matmul(Pminus[k], np.linalg.inv( Pminus[k]+R ))
            # K[k] = np.divide(Pminus[k], ( Pminus[k]+R ))                #equation 19
            xhat[k] = xhatminus[k]+np.matmul(K[k], z[k]-xhatminus[k])    #equation 18

            P[k] = (1-K[k])*Pminus[k]
            print "   measurement = {} {}".format( z[k][0], z[k][1])
            print " => {} update displacement current {},".format(k, xhat[k])
        else:
            print "{}: accel {}, displacement: {} {}".format(k, accel_noise[k], xhatminus[k][0], xhatminus[k][1])
            xhat[k] = xhatminus[k]

        # x = x + delta_x
        x = x + math.sin(k/10) +0.1
        x_truth.append(x)


main()
#plot graph
plotGraph()