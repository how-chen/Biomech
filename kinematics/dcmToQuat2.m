function [ quat ] = dcmToQuat2( R )
% dcmToQuat2  will convert a dcm to a quaternion (q1 is real)
% This function will calculate quaternion from DCM using 
% four different methods for each sample. The most stable method
% (largest normalization factor in denominator) will be used.
%
% input: R (3x3xn)
% ouput: quat (nx4) column 1 is real
% Author: Howard Chen
% Date: 3/20/16
% Update: 3/17/16

sz = size(R);
ln = sz(3); 

quat=zeros(ln,4);

for i=1:ln

    quat(i,1) = 1/2*sqrt(1+R(1,1,i)+R(2,2,i)+R(3,3,i));
    quat(i,2) = 1/2*sqrt(1+R(1,1,i)-R(2,2,i)-R(3,3,i));
    quat(i,3) = 1/2*sqrt(1-R(1,1,i)+R(2,2,i)-R(3,3,i));
    quat(i,4) = 1/2*sqrt(1-R(1,1,i)-R(2,2,i)+R(3,3,i));

    [~, in] = max(quat(i,:));

    if in == 2
        quat(i,3) = 1/4/quat(i,2)*(R(1,2,i)+R(2,1,i));
        quat(i,4) = 1/4/quat(i,2)*(R(1,3,i)+R(3,1,i));
        quat(i,1) = 1/4/quat(i,2)*(R(3,2,i)-R(2,3,i));
    end

    if in == 3
        quat(i,2) = 1/4/quat(i,3)*(R(1,2,i)+R(2,1,i));
        quat(i,4) = 1/4/quat(i,3)*(R(2,3,i)+R(3,2,i));
        quat(i,1) = 1/4/quat(i,3)*(R(1,3,i)-R(3,1,i));
    end

    if in == 4
        quat(i,2) = 1/4/quat(i,4)*(R(1,3,i)+R(3,1,i));
        quat(i,3) = 1/4/quat(i,4)*(R(2,3,i)+R(3,2,i));
        quat(i,1) = 1/4/quat(i,4)*(R(2,1,i)-R(1,2,i));
    end

    if in == 1
        quat(i,2) = 1/4/quat(i,1)*(R(3,2,i)-R(2,3,i));
        quat(i,3) = 1/4/quat(i,1)*(R(1,3,i)-R(3,1,i));
        quat(i,4) = 1/4/quat(i,1)*(R(2,1,i)-R(1,2,i));
    end

end

