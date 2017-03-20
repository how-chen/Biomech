function [quat] = dcmToQuat(DCM)
% dcmToQuat  will convert a dcm to a quaternion (q1 is real)
% input: R (3x3xn)
% ouput: quat (nx4) column 1 is real
% Author: Howard Chen
% Date: 3/20/16
% Update: 3/17/16

ln=length(DCM(1,1,:));
quat=zeros(ln,4);
for i=1:1:ln
    quat(i,1)=0.5.*sqrt(1+DCM(1,1,i)+DCM(2,2,i)+DCM(3,3,i));
    quat(i,2)=(DCM(3,2,i)-DCM(2,3,i))./(4.*quat(i,1));
    quat(i,3)=(DCM(1,3,i)-DCM(3,1,i))./(4.*quat(i,1));
    quat(i,4)=(DCM(2,1,i)-DCM(1,2,i))./(4.*quat(i,1));
end


