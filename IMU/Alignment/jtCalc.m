function [r1,r2]=jtCalc(acc1,gyr1,acc2,gyr2,dT)
%% jtCalc finds the distance between the IMUS on adjacent body segments
% Author: Howard Chen
% GPS & Vehicle Dynamics Lab | Auburn University
% 
% Based on the following papers: 
% Crabolu, M., Pani, D., Raffo, L., Conti, M., Crivelli, P., & Cereatti, A. (2017). 
% In vivo estimation of the shoulder joint center of rotation using magneto-inertial 
% sensors: MRI-based accuracy and repeatability assessment. Biomedical engineering 
% online, 16(1), 34.
%
% Seel, T., Schauer, T., & Raisch, J. (2012, October). Joint axis and position 
% estimation from inertial measurement data by exploiting kinematic constraints. 
% In Control Applications (CCA), 2012 IEEE International Conference on (pp. 45-49).
% IEEE.Chicago	

 
ln = length(acc1);

gyr1Dot = zeros(3,1);
gyr2Dot = zeros(3,1);
K1 = zeros(ln*3,3);
K2 = zeros(ln*3,3); 

%Calculate K 
for i=3:ln-2
    % calculate derivative of gyro angular rate (Seel et al. 
    gyr1Dot(1) = (gyr1(i-2,1)-8*gyr1(i-1,1)+8*gyr1(i+1,1)-gyr1(i+2,1))./(12.*dT); 
    gyr1Dot(2) = (gyr1(i-2,2)-8*gyr1(i-1,2)+8*gyr1(i+1,2)-gyr1(i+2,2))./(12.*dT); 
    gyr1Dot(3) = (gyr1(i-2,3)-8*gyr1(i-1,3)+8*gyr1(i+1,3)-gyr1(i+2,3))./(12.*dT);
    s
    gyr2Dot(1) = (gyr2(i-2,1)-8*gyr2(i-1,1)+8*gyr2(i+1,1)-gyr2(i+2,1))./(12.*dT); 
    gyr2Dot(2) = (gyr2(i-2,2)-8*gyr2(i-1,2)+8*gyr2(i+1,2)-gyr2(i+2,2))./(12.*dT); 
    gyr2Dot(3) = (gyr2(i-2,3)-8*gyr2(i-1,3)+8*gyr2(i+1,3)-gyr2(i+2,3))./(12.*dT); 

    
    %Crabolu et al. eq. 3
    K1(i*3-2:i*3,:) = [ -gyr1(i,2)^2-gyr1(i,3)^2       ,  gyr1(i,1)*gyr1(i,2)-gyr1Dot(3),  gyr1Dot(2) + gyr1(i,1)*gyr1(i,3);...
                         gyr1Dot(3)+gyr1(i,1)*gyr1(i,2), -gyr1(i,1)^2-gyr1(i,3)^2       ,  gyr1(i,2)*gyr1(i,3)-gyr1Dot(1);...
                         gyr1(i,1)*gyr1(i,3)-gyr1Dot(2),  gyr1Dot(1)+gyr1(i,2)*gyr1(i,3), -gyr1(i,1)^2-gyr1(i,2)^2]; 
        
    K2(i*3-2:i*3,:) = [ -gyr2(i,2)^2-gyr2(i,3)^2       ,  gyr2(i,1)*gyr2(i,2)-gyr2Dot(3),  gyr2Dot(2) + gyr2(i,1)*gyr2(i,3);...
                         gyr2Dot(3)+gyr2(i,1)*gyr2(i,2), -gyr2(i,1)^2-gyr2(i,3)^2       ,  gyr2(i,2)*gyr2(i,3)-gyr2Dot(1);...
                         gyr2(i,1)*gyr2(i,3)-gyr2Dot(2),  gyr2Dot(1)+gyr2(i,2)*gyr2(i,3), -gyr2(i,1)^2-gyr2(i,2)^2];    
end

%delete zeros from data
K1(1:6,:) = [];
K1(end-5:end,:) = [];

K2(1:6,:) = [];
K2(end-5:end,:) = [];

acc1(1:2,:) = [];
acc1(end-1:end,:) = [];

acc2(1:2,:) = [];
acc2(end-1:end,:) = [];

%% Gauss Newton Calc
ln = length(acc2); 
e = zeros(ln,1); %error
J = zeros(ln,6); %jacobian
r = zeros(6,1);  

r_prev = [0 0 0 .1 .1 .1]';

j=1; 
while max(abs(r-r_prev)) > 1.e-6 && j < 1000
    
    for i=1:ln
        %Crabolu et al. eq. 6
        J(i,:) = [-((acc1(i,:)'-K1(i*3-2:i*3,:)*r(1:3))'*K1(i*3-2:i*3,:))/norm(acc1(i,:)'-K1(i*3-2:i*3,:)*r(1:3)),...
                   ((acc2(i,:)'-K2(i*3-2:i*3,:)*r(4:6))'*K2(i*3-2:i*3,:))/norm(acc1(i,:)'-K2(i*3-2:i*3,:)*r(4:6))]; 
    
        %Crabolu et al. eq. 7
        e(i) = norm(acc1(i,:)'-K1(i*3-2:i*3,:)*r(1:3)) - norm(acc2(i,:)'-K2(i*3-2:i*3,:)*r(4:6)); 
    end
    
    r_prev = r;
    r = r - (J'*J)^-1*J'*e;

    j = j+1; 
end

r1 = r(1:3);
r2 = r(4:6);