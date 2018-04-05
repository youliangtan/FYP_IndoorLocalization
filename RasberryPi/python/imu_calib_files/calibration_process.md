* PROCESS OF CALIBRATION *

1) 3 steps in total magnetic max min, ellipsoid mag offset, and accel

2) for ellipsoid>>>

use "RTIMULibCal" in dir: /home/youliang/RTIMULib2/RTEllipsoidFit
 
Due to the dependency of RTEllipsoid.m matlab script 

output file, copy the results to the .ini file


3) for accel>

continue with the accel calibration only by using the previous calibrated mag calibration value
continue using the "RTIMULibCal" or "RTIMULibDemoGL"


4) Done with calibration, copy ini file to python fusion script to run program
