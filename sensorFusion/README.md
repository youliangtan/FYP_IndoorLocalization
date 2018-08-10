# Notes

> kalman_v1.py, kalman_v2.py and kalman_backup.py:
- These 3 scripts are just for testing purpose. It will use MatPlotlib or IMU accel data for visualization of the algorithm.

> fusion_v1 and fusion_v2.py:
- use ros sub and ros pub, while fusing cam and imu data together and output the kalman filtered value to user

> fusion_v3.py
- finalized fusion script. Using Encoder odometry value as kalman filter measurement update. By using encoder odometry is way better than IMU odometry, due to less driftting
