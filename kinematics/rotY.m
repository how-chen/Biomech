function [out]=rotY(theta)
  % 3x3 Rotation Matrix around Y-axis (right-handed) 
  % Author: Howard Chen
  % See: https://en.wikipedia.org/wiki/Rotation_matrix
  
    out=[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
end
