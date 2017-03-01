function [ quat ] = quatY( theta )
%QUATX Summary of this function goes here
%   Detailed explanation goes here
quat(:,1)=cos(theta./2);
quat(:,2)=zeros(length(theta),1);
quat(:,3)=sin(theta./2);
quat(:,4)=zeros(length(theta),1);
end