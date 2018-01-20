function [ quat ] = eulerToQuat( euler )
%EULERTOQUAT converts from quat to ypr
%   inputs:
%   euler (nx3)
%      col 1- yaw
%      col 2- pitch
%      col 3- euler(:,3)
%
%   output:
%   quaternion (nx4)
%      col 1- 'real'
%      col 2 to 4- 'imaginary'

quat(:,1) = cos(euler(:,1)./2).*cos(euler(:,2)./2).*cos(euler(:,3)./2)+sin(euler(:,1)./2).*sin(euler(:,2)./2).*sin(euler(:,3)./2);
quat(:,2) = cos(euler(:,1)./2).*cos(euler(:,2)./2).*sin(euler(:,3)./2)-sin(euler(:,1)./2).*sin(euler(:,2)./2).*cos(euler(:,3)./2);
quat(:,3) = cos(euler(:,1)./2).*sin(euler(:,2)./2).*cos(euler(:,3)./2)+sin(euler(:,1)./2).*cos(euler(:,2)./2).*sin(euler(:,3)./2);
quat(:,4) = sin(euler(:,1)./2).*cos(euler(:,2)./2).*cos(euler(:,3)./2)-cos(euler(:,1)./2).*sin(euler(:,2)./2).*sin(euler(:,3)./2);

end

