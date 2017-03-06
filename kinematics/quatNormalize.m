function [ out ] = quatNormalize( quat )
%QUATNORMALIZE normalizes the quaternion to a unit vector 
%   input: nx4 quaternion to be normalized
%   outout: nx4 normalized quaternion 

%   Author: Howard Chen
%   Last Update: 3/6/2017

quatNorm=sqrt(quat(:,1).^2+quat(:,2).^2+quat(:,3).^2+quat(:,4).^2);

out(:,1)=quat(:,1)./quatNorm;
out(:,2)=quat(:,2)./quatNorm;
out(:,3)=quat(:,3)./quatNorm;
out(:,4)=quat(:,4)./quatNorm;
end

