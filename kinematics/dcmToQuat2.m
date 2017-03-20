function [ quat ] = dcmToQuat2( R )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here

q = zeros(4,1);
quat=zeros(1,4);

q(4) = 1/2*sqrt(1+R(1,1)+R(2,2)+R(3,3));
q(1) = 1/2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
q(2) = 1/2*sqrt(1-R(1,1)+R(2,2)-R(3,3));
q(3) = 1/2*sqrt(1-R(1,1)-R(2,2)+R(3,3));

[k, in] = max(q);

if in == 1
    q(2) = 1/4/q(1)*(R(1,2)+R(2,1));
    q(3) = 1/4/q(1)*(R(1,3)+R(3,1));
    q(4) = 1/4/q(1)*(R(3,2)-R(2,3));
end
if in == 2
    q(1) = 1/4/q(2)*(R(1,2)+R(2,1));
    q(3) = 1/4/q(2)*(R(2,3)+R(3,2));
    q(4) = 1/4/q(2)*(R(1,3)-R(3,1));
end
if in == 3
    q(1) = 1/4/q(3)*(R(1,3)+R(3,1));
    q(2) = 1/4/q(3)*(R(2,3)+R(3,2));
    q(4) = 1/4/q(3)*(R(2,1)-R(1,2));
end
if in == 4
    q(1) = 1/4/q(4)*(R(3,2)-R(2,3));
    q(2) = 1/4/q(4)*(R(1,3)-R(3,1));
    q(3) = 1/4/q(4)*(R(2,1)-R(1,2));
end

quat(1)=q(4);
quat(2:4)=q(1:3);


end

