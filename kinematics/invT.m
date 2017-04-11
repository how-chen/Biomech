function [ tINV ] = invT( t )
%INVT Summary of this function goes here
%   Detailed explanation goes here

ln =length(t(1,1,:));
tINV=zeros(4,4,ln);
for i=1:ln
    tINV(1:3,1:3,i) = t(1:3,1:3,i)';
    tINV(1:3,4,i) = -t(1:3,1:3,i)'*t(1:3,4,i);
    tINV(4,4,i) = 1;
end
end

