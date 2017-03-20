function [ out ] = rotZt( theta )
%ROTZ Summary of this function goes here
%   Detailed explanation goes here
out=[cos(theta),-sin(theta),0,0;
    sin(theta),cos(theta),0,0;
    0,0,1,0;
    0,0,0,1];

end

