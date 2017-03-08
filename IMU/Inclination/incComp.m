function [ quat ] = incComp(gyro, accel, freq, alpha)
%INCCOMP calculates inclination from accel and gyro measurements 
%   calculations is done in yaw-pitch-roll and converted to quaternions
%   input: accel- accelerometer measurements
%          gyro- gyroscope mearements
%          freq- sampling frequency
%          alpha- filter coefficient (1 = all accel)
%                 alpha(1): pitch 
%                 alpha(2): roll
%   output: q- quaternion measurements (can be converted back to Euler) 
% 
%   Author: Howard Chen

ln = length(accel); 
quat = zeros(ln,4); 

dT = 1/freq; 

%calculate pitch and roll from accelerometer
aPitch = atan(-accel(:,1)./sqrt(accel(:,2).^2+accel(:,3).^2));
aRoll = atan(accel(:,2)./accel(:,3));

cPitch = aPitch(1);
cRoll = aRoll(1); 
for i=1:ln
    cPitch = (1-alpha(1))*(cPitch+(gyro(i,2)*cos(cRoll)-gyro(i,3)*sin(cRoll))*dT)+alpha(1)*aPitch(i);  
    cRoll = (1-alpha(2))*(cRoll+(gyro(i,1)+gyro(i,2)*sin(cRoll)*tan(cPitch)+gyro(i,3)*cos(cRoll)*tan(cPitch))*dT)+alpha(2)*aRoll(i);

    %Convert to quaternion    
    quat(i,1)=cos(cPitch./2).*cos(cRoll./2);
    quat(i,2)=cos(cPitch./2).*sin(cRoll./2);
    quat(i,3)=sin(cPitch./2).*cos(cRoll./2);
    quat(i,4)=-sin(cPitch./2).*sin(cRoll./2);
end


