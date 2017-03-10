function [ out ] = modLKF(gyro, accel, freq, gyroNoise, gyroBias, accelNoise,ca,cb)
%LKF Linear kalman filter
%   Adopted from the code in the following paper: 
%   Ligorio, G., & Sabatini, A. M. (2015). A novel Kalman filter for human motion 
%   tracking with an inertial-based dynamic inclinometer. 
%   IEEE Transactions on Biomedical Engineering, 62(8), 2033-2043.
%   
%   -The following adaptions were made: 
%   -added a gyroscope bias random walk model
%   -used first order approximation for matrix exponential
%   -Used EKF filter structure
%   
%   measureData: n x 3 array contatining vector measurements (accelerometer or magnetometer)
%   processData: n x 3 array containing angular rate measurements (gyroscope)
%   measureNoise: measurement noise
%   processNoise: process noise
%   processBias: gyroscope bias
%   freq: Sampling frequency (Hz)
%   ca, cb: Gauss Markov Parameters
%   output: n x 9 vector 
%           col 1 to 3: gravity (local frame) 
%           col 4 to 6: acceleration (local frame) 
%           col 7 to 9: acceleration (local frame)
%   Author: Howard Chen

% Sampling period
dT = 1/freq; 

ln = length(accel); 
% Process Covariance Matrix
Q = eye(9);
Q(1:3,1:3) = gyroNoise^2.*eye(3); 
Q(7:9,7:9) = gyroBias^2.*eye(3);

% Measurement Covariance Matrix
R = accelNoise^2.*eye(3); 

A = zeros(9,9);
P = zeros(9,9);
W = zeros(9,9); 
H = [eye(3),eye(3),zeros(3,3)]; 
x = zeros(9,1); 
x(1:3) = accel(1,:)'; 
out = zeros(ln,9);

for i=1:ln; 
    A(1:3,:) = [eye(3)-skew((gyro(i,:)-x(7:9)').*dT), zeros(3,3),-skew(x(1:3).*dT)]; 
    A(4:6,:) = [zeros(3,3), ca.*eye(3), zeros(3,3)];
    A(7:9,:) = [zeros(3,3), zeros(3,3), eye(3)]; 

    x = [A(1:6,1:6)*x(1:6);x(7:9)]; 
    W(1:3,:) = [skew(x(1:3).*dT), zeros(3,3), zeros(3,3)];
    W(4:6,:) = [zeros(3,3), cb.*eye(3),zeros(3,3)]; 
    W(7:9,:) = [zeros(3,3),zeros(3,3),dT.*eye(3)]; 

    P = A*P*A' + W*Q*W';
    K = P*H'*(H*P*H'+R)^-1;
    x = x+K*(accel(i,:)'-H*x);
    P = (eye(9)-K*H)*P;
    out(i,:) = x'; 
end

end

function [ wx ] = skew( a )
    % Skew symmetric matrix
    wx = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
end

