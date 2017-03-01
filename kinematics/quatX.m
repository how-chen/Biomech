function [ quat ] = quatX( theta )
%QUATX right-handed rotation around x-axis in quaternions
%   quat: n x 4 array (1st column is real) 
%   Author: Howard Chen
%   Affiliation: University of Iowa

quat(:,1)=cos(theta./2);
quat(:,2)=sin(theta./2);
quat(:,3)=zeros(length(theta),1);
quat(:,4)=zeros(length(theta),1);

end

