function [ q ] = accelMag( accel,mag)
%ACCELMAG calculates orientation using accelerometer & magnetometers
%   calculations is done in yaw-pitch-roll and converted to quaternions
%   accel: nx3 vector with accelerometer measurements (gravity vector)
%   mag: nx3 vector with magnetometer measurements (magetic north vector) 
%
%   Author: Howard Chen
%   Date Created: 3/2/2017

% calculate pitch and roll from accel

%pitch
pitch = atan2(-accel(:,1),sqrt(accel(:,2).^2+accel(:,3).^2));

%roll
roll = atan2(accel(:,2),accel(:,3));

% calculate yaw from mag and accel
yaw=atan2(mag(:,3).*sin(roll)-mag(:,2).*cos(roll),mag(:,1).*cos(pitch)+mag(:,2).*sin(pitch).*sin(roll)+mag(:,3).*sin(pitch).*cos(roll));

% convert ypr to quaternion
q(:,1) = cos(yaw./2).*cos(pitch./2).*cos(roll./2)+sin(yaw./2).*sin(pitch./2).*sin(roll./2);
q(:,2) = cos(yaw./2).*cos(pitch./2).*sin(roll./2)-sin(yaw./2).*sin(pitch./2).*cos(roll./2);
q(:,3) = cos(yaw./2).*sin(pitch./2).*cos(roll./2)+sin(yaw./2).*cos(pitch./2).*sin(roll./2);
q(:,4) = sin(yaw./2).*cos(pitch./2).*cos(roll./2)-cos(yaw./2).*sin(pitch./2).*sin(roll./2);
end