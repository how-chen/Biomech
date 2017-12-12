function [ r ] = pendLevArmCalc(accel,gyro,quat,dT)
%FINDR Solves for Pendulum Lever Arm
%   Work based on following:
%   Crabolu, M., Pani, D., Raffo, L., & 
%   Cereatti, A. (2016). Estimation of the 
%   center of rotation using wearable 
%   magneto-inertial sensors. Journal of 
%   biomechanics, 49(16), 3928-3933.
%
%   Implemented by Howard Chen
%   GPS & Vehicle Dynamics Lab | Auburn University

ln = length(gyro); 
a=zeros(ln*3,1); 
K=zeros(ln*3,3); 

%rotate gravity vector from global to local coordinate frame
gL = quatMultiply(quatConj(quat),quatMultiply([zeros(ln,3),ones(ln,1).*9.81],quat)); %gravity local
gL(:,1)=[];  %delete first column of data

aL = accel-gL; %accel Local (subtract gravity) 


for i=3:ln-2
    % calculate derivative of gyro angular rate 
    wx_dot = (gyro(i-2,1)-8*gyro(i-1,1)+8*gyro(i+1,1)-gyro(i+2,1))./(12.*dT); 
    wy_dot = (gyro(i-2,2)-8*gyro(i-1,2)+8*gyro(i+1,2)-gyro(i+2,2))./(12.*dT); 
    wz_dot = (gyro(i-2,3)-8*gyro(i-1,3)+8*gyro(i+1,3)-gyro(i+2,3))./(12.*dT); 
    gyro(i,1) = gyro(i,1);
    gyro(i,2) = gyro(i,2);
    gyro(i,3) = gyro(i,3);

    K(i*3-2:i*3,:) = [-gyro(i,2)^2-gyro(i,3)^2, gyro(i,1)*gyro(i,2)-wz_dot, wy_dot + gyro(i,1)*gyro(i,3);...
                      wz_dot+gyro(i,1)*gyro(i,2), -gyro(i,1)^2-gyro(i,3)^2, gyro(i,2)*gyro(i,3)-wx_dot;...
                      gyro(i,1)*gyro(i,3)-wy_dot, wx_dot+gyro(i,2)*gyro(i,3), -gyro(i,1)^2-gyro(i,2)^2]; 
    
    %align accel by column
    a(i*3-2:i*3) = aL(i,:)';

end

%Delete the first two data epocs (due to time derivative)
K(1:6,:) = [];
K(end-5:end,:) = []; 
a(1:6,:) = [];
a(end-5:end,:) = []; 

%Pseudoinverse to calculate lever arm 
r =  pinv(K)*a; 
