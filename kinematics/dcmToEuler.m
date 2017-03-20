function [ out ] = dcmToEuler( R )
% dcmToEuler  will convert a dcm to euler (Z-Y-X, (yaw pitch roll)
% input: R (3x3xn)
% ouput: colum 1- yaw, column 2- pitch, column 3- roll
% Author: Howard Chen
% Date: 3/20/16
% Update: 3/17/16

out=zeros(length(R(1,1,:)),3);

for i=1:1:length(R(1,1,:))
    %Yaw
    out(i,1)=atan2(R(2,1,i),R(1,1,i));
    %pitch
    out(i,2)=-asin(R(3,1,i));
    %Roll
    out(i,3)=atan2(R(3,2,i),R(3,3,i));
end



end

