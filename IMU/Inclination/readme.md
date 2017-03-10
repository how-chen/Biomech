# Introduction
  These functions are used to calculate inclination measurements (pitch and roll) angles. The output for all functions is in quaternions to remove ambiguity. For these functions, the first element of the quaternion vector is the 'real' or 'scalar' component. It is not uncommon for the real component to be define as the last component of the quaternion rotation vector. All code was tested to work with the APDM 'OPAL' sensor. Other sensors and chips may have a different coordinate system convention and requires modification. 

  Each function in this folder is designed to function independently from one another, resulting in redundant code. 
  
## incAccel
This function calculates (pitch and roll) angles given accelerometer measurements. 

## LKF
Direct implementation of the Kalman Filter within the following paper: 
Ligorio, G., & Sabatini, A. M. (2015). A novel Kalman filter for human motion tracking with an inertial-based dynamic inclinometer. IEEE Transactions on Biomedical Engineering, 62(8), 2033-2043.

## incMadgwick
Implementation of the Kalman Filter within the following paper. Modifications were made from the supplied code to provide coding consistencies. The magnetometer portion was removed to improve computation speeds, even though keeping it would have no effects on heading measurements. 
Madgwick, S. O. (2010). An efficient orientation filter for inertial and inertial/magnetic sensor arrays. Report x-io and University of Bristol (UK).
