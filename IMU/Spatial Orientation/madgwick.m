function quat = madgwick(accel,gyro,mag,freq,beta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quaternion-based Complementary Filter 
% Program will derive orientation using gyroscope, magnetometer, and 
% accelerometer from the APDM Opal IMU in quaternion using Madgewick's
% Algorithm
%
% See: 
% Madgwick, S. O. (2010). An efficient orientation filter for inertial 
% and inertial/magnetic sensor arrays. Report x-io and University of 
% Bristol (UK).
% 
% Implementation of Madgwick's Complementary Filter
% Author: Howard Chen
% Affiliation: University of Iowa College of Public Health
%
% Parameters: accel, gyro, mag, freq, beta
% output: quaternion (q0 is real) 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Parameters
dT=1/freq;

%Accelerometer
Ax=accel(:,1)./sqrt(accel(:,1).^2+accel(:,2).^2+accel(:,3).^2);
Ay=accel(:,2)./sqrt(accel(:,1).^2+accel(:,2).^2+accel(:,3).^2);
Az=accel(:,3)./sqrt(accel(:,1).^2+accel(:,2).^2+accel(:,3).^2);

%Gyroscope
Gx=gyro(:,1);
Gy=gyro(:,2);
Gz=gyro(:,3); 

%Magnetometer
Mx=mag(:,1)./sqrt(mag(:,1).^2+mag(:,2).^2+mag(:,3).^2);
My=mag(:,2)./sqrt(mag(:,1).^2+mag(:,2).^2+mag(:,3).^2);
Mz=mag(:,3)./sqrt(mag(:,1).^2+mag(:,2).^2+mag(:,3).^2);

%Initialization
n=length(accel);
quat = zeros(n,4);
q = [1 0 0 0];


for i=1:1:n
  % Reference direction of Earth's magnetic field   
   h=[(1-2*(q(3)^2+q(4)^2))*Mx(i)+ 2*(q(2)*q(3)-q(1)*q(4))*My(i)+  2*(q(1)*q(3)+q(2)*q(4))*Mz(i);
       2*(q(2)*q(3)+q(1)*q(4))*Mx(i)+ (1-2*(q(2)^2+q(4)^2))*My(i)+ 2*(q(3)*q(4)-q(1)*q(2))*Mz(i);
       2*(q(2)*q(4)-q(1)*q(3))*Mx(i)+ 2*(q(1)*q(2)+q(3)*q(4))*My(i)+ (1-2*(q(2)^2+q(3)^2))*Mz(i)];
    
   b = [norm([h(1) h(2)]) 0 h(3)];

   % Gradient decent algorithm corrective step
   F = [2*(q(2)*q(4) - q(1)*q(3)) - Ax(i)
        2*(q(1)*q(2) + q(3)*q(4)) - Ay(i)
        2*(0.5 - q(2)^2 - q(3)^2) - Az(i)
        2*b(1)*(0.5 - q(3)^2 - q(4)^2) + 2*b(3)*(q(2)*q(4) - q(1)*q(3)) - Mx(i)
        2*b(1)*(q(2)*q(3) - q(1)*q(4)) + 2*b(3)*(q(1)*q(2) + q(3)*q(4)) - My(i)
        2*b(1)*(q(1)*q(3) + q(2)*q(4)) + 2*b(3)*(0.5 - q(2)^2 - q(3)^2) - Mz(i)];
            
   J = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
         2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
        0,                         -4*q(2),                    -4*q(3),                         0
        -2*b(3)*q(3),               2*b(3)*q(4),               -4*b(1)*q(3)-2*b(3)*q(1),       -4*b(1)*q(4)+2*b(3)*q(2)
        -2*b(1)*q(4)+2*b(3)*q(2),	2*b(1)*q(3)+2*b(3)*q(1),	2*b(1)*q(2)+2*b(3)*q(4),       -2*b(1)*q(1)+2*b(3)*q(3)
        2*b(1)*q(3),                2*b(1)*q(4)-4*b(3)*q(2),	2*b(1)*q(1)-4*b(3)*q(3),        2*b(1)*q(2)];
        
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
               
    % Compute rate of change of quaternion
    qDot=q+0.5.*[-q(2)*Gx(i)-q(3)*Gy(i)-q(4)*Gz(i);
                  q(1)*Gx(i)+q(3)*Gz(i)-q(4)*Gy(i);
                  q(1)*Gy(i)-q(2)*Gz(i)+q(4)*Gx(i);
                  q(1)*Gz(i)+q(2)*Gy(i)-q(3)*Gx(i)]'- beta * step'; 
          
    % Integrate to yield quaternion
    q = q + qDot * dT;
    
    %Normalization and Output
    q = q / norm(q);
    quat(i,1)=q(1);
    quat(i,2)=q(2);
    quat(i,3)=q(3);
    quat(i,4)=q(4);
end


        