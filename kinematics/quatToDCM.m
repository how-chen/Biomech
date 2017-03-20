function [R] = quatToDCM(q)
%quatToDCM converts from quaternion vector to DCM
% input: quaternion vector (nx4) first element is real
% output: Rotation matrix (3x3xn)
%
% Author: Howard Chen

    sz = size(q);
    ln = sz(1);
    R = zeros(3,3,ln);
    
    for i=1:ln
        R(1,1,i)= q(i,1)^2+q(i,2)^2-q(i,3)^2-q(i,4)^2;
        R(1,2,i)=-2*q(i,1)*q(i,4)+2*q(i,2)*q(i,3);
        R(1,3,i)= 2*q(i,1)*q(i,3)+2*q(i,2)*q(i,4);
        R(2,1,i)= 2*q(i,1)*q(i,4)+2*q(i,2)*q(i,3);
        R(2,2,i)= q(i,1)^2-q(i,2)^2+q(i,3)^2-q(i,4)^2;
        R(2,3,i)=-2*q(i,1)*q(i,2)+2*q(i,3)*q(i,4);
        R(3,1,i)=-2*q(i,1)*q(i,3)+2*q(i,2)*q(i,4);
        R(3,2,i)=2*q(i,1)*q(i,2)+2*q(i,3)*q(i,4);
        R(3,3,i)=q(i,1)^2-q(i,2)^2-q(i,3)^2+q(i,4)^2;
    end

end


