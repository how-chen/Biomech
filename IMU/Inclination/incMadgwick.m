function quat = incMadgwick(gyro,accel,freq,beta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quaternion-based Complementary Filter (2nd order)
% Program will derive inclination using gyroscope and accelerometer
% from the APDM Opal IMU in quaternion using Madgewick's Algorithm
%
% Implementation of Madgwick's Complementary Filter
% Author: Howard Chen
% Affiliation: University of Iowa College of Public Health
%
% Parameters: accel, gyro, mag, freq, beta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Parameters
samplePeriod=1/freq;

%Accelerometer
Ax=accel(:,1)./sqrt(accel(:,1).^2+accel(:,2).^2+accel(:,3).^2);
Ay=accel(:,2)./sqrt(accel(:,1).^2+accel(:,2).^2+accel(:,3).^2);
Az=accel(:,3)./sqrt(accel(:,1).^2+accel(:,2).^2+accel(:,3).^2);

%Initialization
n=length(accel);
quat = zeros(n,4);
q = [1 0 0 0];


for i=1:1:n
   % Gradient decent algorithm corrective step
   F = [2*(q(2)*q(4) - q(1)*q(3)) - Ax(i)
        2*(q(1)*q(2) + q(3)*q(4)) - Ay(i)
        2*(0.5 - q(2)^2 - q(3)^2) - Az(i)];
            
   J = [-2*q(3),  2*q(4), -2*q(1), 2*q(2)
         2*q(2),  2*q(1),  2*q(4), 2*q(3)
         0     , -4*q(2), -4*q(3), 0];
        
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
               
    % Compute rate of change of quaternion
    qDot=q+0.5.*[-q(2)*gyro(i,1)-q(3)*gyro(i,2)-q(4)*gyro(i,3);
                  q(1)*gyro(i,1)+q(3)*gyro(i,3)-q(4)*gyro(i,2);
                  q(1)*gyro(i,2)-q(2)*gyro(i,3)+q(4)*gyro(i,1);
                  q(1)*gyro(i,3)+q(2)*gyro(i,2)-q(3)*gyro(i,1)]'- beta * step'; 
          
    % Integrate to yield quaternion
    q = q + qDot * samplePeriod;
    
    %Normalization and Output
    q = q ./ sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
    quat(i,1)=q(1);
    quat(i,2)=q(2);
    quat(i,3)=q(3);
    quat(i,4)=q(4);
end


        