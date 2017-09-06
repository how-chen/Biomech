function [ quat ] = gyroToQuat(gyro, initial, dT)
%GYROTOQUAT - calculates orientation using gyroscope-derived angular rate measurements 
%
% Syntax:  [quat]  = gyroToQuat(omega, initial, dT )
%
% Inputs:
%    gyro - gyroscope measurements (n x 3) 
%    initial - initial orientation ([1 0 0 0] for dead reckoning)
%    dT - sampling period (seconds)
%
% Outputs:
%    quat - quaternion orientation [w x y z] (n x 4) 
%
% Example: 
%    quat = gyroToQuat(gyro,[1 0 0 0], 1/150) 
%
% Citation: Equations 3 to 6 from 
% Brodie, M. A., Walmsley, A., & Page, W. (2008). 
% Dynamic accuracy of inertial measurement units during simple 
% pendulum motion. Computer methods in biomechanics and biomedical 
% engineering, 11(3), 235-242.
%
% Author: Howard Chen | Postdoctoral Fellow
% GPS & Vehicle Dynamics Laboratory
% email address: hzc0074@auburn.edu 
% Last revision: 9/6/2017

%------------- BEGIN CODE --------------

quat=zeros(length(gyro),4);
quat(1,:)=initial;

for i=1:1:length(gyro)
    omegaNorm=sqrt(gyro(i,1).^2+gyro(i,2).^2+gyro(i,3).^2);
    dTheta=omegaNorm.*dT;
    Ux=gyro(i,1)/omegaNorm;
    Uy=gyro(i,2)/omegaNorm;
    Uz=gyro(i,3)/omegaNorm;
    dQ=[cos(dTheta/2),Ux*sin(dTheta/2),Uy*sin(dTheta/2),Uz*sin(dTheta/2)];
    quat(i+1,:)=quatMultiply(quat(i,:),dQ);
    quat(i+1,:)=quatNormalize(quat(i+1,:));
end

%------------- END OF CODE --------------