function [ out ] = LKF(processData,measureData, freq, processNoise, measureNoise,ca,cb)
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
%   dT: Sampling period (seconds)
%   ca, cb: Gauss Markov Parameters
%   output: n x 6 vector 
%           col 1 to 3: gravity (local frame) 
%           col 4 to 6: acceleration (local frame) 
%
%   Author: Howard Chen

% Sampling period
dT = 1/freq; 

% Process Covariance Matrix
Q = eye(6);
Q(1:3,1:3) = processNoise^2.*eye(3); 

% Measurement Covariance Matrix
R = accelNoise^2.*eye(3); 

A = zeros(6,6);
P = zeros(6,6);
W = zeros(6,6); 
H = [eye(3),eye(3)]; 
x = zeros(6,1); 
x(1:3) = measurementData(1,:)'; 
out = zeros(length(measurementData),6);

for i=1:length(measurementData); 
    A(1:3,1:3) = expm(- skew(processData(i,:)).*dT);  
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

function [ wx ] = skew( a )
    % Skew symmetric matrix
    wx = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
end

