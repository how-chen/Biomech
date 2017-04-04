function [ quat ] = gyroToQuat(omega, initial, freq )
%GYROTOQUAT derives quaternion from angular velocity and sampling rate
% input- omega: gyroscope angular velocity
%        initial: initial orientation (quaternion)
%
% Author: Howard Chen


quat=zeros(length(omega),4);
quat(1,:)=initial;

for i=1:1:length(omega);
    omegaNorm=sqrt(omega(i,1).^2+omega(i,2).^2+omega(i,3).^2);
    dTheta=omegaNorm/freq;
    Ux=omega(i,1)/omegaNorm;
    Uy=omega(i,2)/omegaNorm;
    Uz=omega(i,3)/omegaNorm;
    dQ=[cos(dTheta/2),Ux*sin(dTheta/2),Uy*sin(dTheta/2),Uz*sin(dTheta/2)];
    quat(i+1,:)=quatMultiply(quat(i,:),dQ);
    quat(i+1,:)=quatNormalize(quat(i+1,:));
end

