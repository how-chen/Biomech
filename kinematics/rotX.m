function [out] = rotX(theta)
  % Author: Howard Chen
  % 3x3 Rotation Matrix around X-axis (right-handed) 
  % See: https://en.wikipedia.org/wiki/Rotation_matrix
  
  out=[1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
end
