function [ out ] = LKF(measureData,processData,dT, measureNoise,processNoise,ca,cb)
%LKF Linear kalman filter
%   Implementation of the code in the following paper: 
%   Ligorio, G., & Sabatini, A. M. (2015). A novel Kalman filter for human motion 
%   tracking with an inertial-based dynamic inclinometer. 
%   IEEE Transactions on Biomedical Engineering, 62(8), 2033-2043.
%
%   measureData: n x 3 array contatining vector measurements (accelerometer or magnetometer)
%   processData: n x 3 array containing angular rate measurements (gyroscope)
%   measureNoise: measurement noise
%   processNoise: process noise
%   ca, cb: Gauss Markov Parameters
%
%   Author: Howard Chen

Q = eye(6);
Q(1:3,1:3) = processNoise^2.*eye(3); 
R = accelNoise^2.*eye(3); 

A = zeros(6,6);
P = zeros(6,6);
W = zeros(6,6); 
H = [eye(3),eye(3)]; 
out = zeros(length(accel),6);

x = [0 0 0 0 0 0]';
x(1:3) = measurementData(1,:)'; 
for i=1:length(measurementData); 
    A(1:3,1:3) = expm(- skew(processData(i,:)).*dT);  
    %A(1:3,1:3) = eye(3) - skew(processData(i,:).*dT);
    A(4:6,4:6) = ca.*eye(3); 
    x = A*x; 

    W(1:3,1:3) = skew(x(1:3).*dT); 
    W(4:6,4:6) = cb.*eye(3); 
    P = A*P*A'+W*Q*W'; 

    K = P*H'*(H*P*H'+R)^-1; 
    x = x+K*(measurementData(i,:)'-H*x); 
    P = (eye(6)-K*H)*P; 
    out(i,:) = x'; 
end

