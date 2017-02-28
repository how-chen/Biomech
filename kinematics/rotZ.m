function [ out ] = rotZ(theta)
  % 3x3 Rotation Matrix around Z-axis (right-handed) 
  % Author: Howard Chen
  % See: https://en.wikipedia.org/wiki/Rotation_matrix
  
  
out=[cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
end

