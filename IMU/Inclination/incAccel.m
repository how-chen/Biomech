function [ q ] = incAccel(accel)
%INCACCEL calculates inclination usin accelerometer measurements 
%   calculations is done in yaw-pitch-roll and converted to quaternions
%   input: accel- accelerometer measurements
%   output: q- quaternion measurements (can be converted back to Euler) 
% 
%   Author: Howard Chen

%calculate pitch and roll from accel
pitch=atan2(-accel(:,1),sqrt(accel(:,2).^2+accel(:,3).^2)); %pitch
roll=atan2(accel(:,2),accel(:,3));  %roll

%Convert to quaternion    
q(:,1)=cos(pitch./2).*cos(roll./2);
q(:,2)=cos(pitch./2).*sin(roll./2);
q(:,3)=sin(pitch./2).*cos(roll./2);
q(:,4)=-sin(pitch./2).*sin(roll./2);