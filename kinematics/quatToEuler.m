function [out]=quatToEuler(q)
%QUATTOEULER converts from quaternion vector to Euler rotation
%sequence of yaw-pitch-roll
% input: quaternion rotation vector (nx4) q1 is real
% output: euler rotation sequence
%         col 1: yaw, col 2: pitch, col 3: roll
%
% Author: Howard Chen

%yaw
out(:,1)=atan2(2.*(q(:,1).*q(:,4)+q(:,2).*q(:,3)),(q(:,1).^2+q(:,2).^2-q(:,3).^2-q(:,4).^2));
%pitch
out(:,2)=asin(2.*(q(:,1).*q(:,3)-q(:,4).*q(:,2)));
%roll
out(:,3)=atan2(2.*(q(:,1).*q(:,2)+q(:,3).*q(:,4)),(q(:,1).^2-q(:,2).^2-q(:,3).^2+q(:,4).^2));

end
