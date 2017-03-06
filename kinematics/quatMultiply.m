function [ qOut ] = quatMultiply( q1, q2 )
% This function multiplies 2 quaternion vectors
% Assumes the 'real' component is the first element
% Quaternion vectors are n x 4 elements 
%
% Author: Howard Chen
% Created: 2/25/2017

qOut=[q1(:,1).*q2(:,1)-q1(:,2).*q2(:,2)-q1(:,3).*q2(:,3)-q1(:,4).*q2(:,4),... 
      q1(:,1).*q2(:,2)+q1(:,2).*q2(:,1)+q1(:,3).*q2(:,4)-q1(:,4).*q2(:,3),...
      q1(:,1).*q2(:,3)-q1(:,2).*q2(:,4)+q1(:,3).*q2(:,1)+q1(:,4).*q2(:,2),...
      q1(:,1).*q2(:,4)+q1(:,2).*q2(:,3)-q1(:,3).*q2(:,2)+q1(:,4).*q2(:,1)]; 

end

