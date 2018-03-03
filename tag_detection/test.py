import numpy as np
import transformation
import math

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    # assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

# arr = np.ones((3,3))
# arr = ([[1,2,3], [5,6,7], [8,9,10]], dtype=np.int64)

# z = np.matrix('0.8132 0 -113; 0.084 0.985 -393; -0.001 -0.00015 1')
# z = np.matrix('0.8472927 -0.5311051 -0.0047381; -0.5310975 -0.8471202 -0.0179653; 0.0055277 0.0177382 -0.9998274')
# z = np.matrix( '1.267283  0 -3.14286289e+02; -4.52340192e-02 1.67818  -4.40212952e+02; 2.16668150e-04   1.58543613e-04   1')
z = np.matrix('1.267283  0 -314; -0.04 1.67818  -440; 0.0002 0.000158 1')
# z = z.transpose()
print(z)

# print(transformation.euler_from_matrix(y))
# print(rotationMatrixToEulerAngles(y))
# print(rotationMatrixToEulerAngles(z))
print(transformation.euler_matrix(z))

print("_____qua - mat_______")
qua = transformation.quaternion_from_matrix(z)
print(qua)
mat = transformation.quaternion_matrix(qua)   #normalize data rot -> qua -> rot (normalized) -> euler
print(mat)
print(transformation.euler_from_quaternion(qua))