function [ q ] = accelMag( accel,mag )
%ACCELMAG calculates orientation using accel & mag
%   calculations is done in ypr and converted to quaternions

n=length(accel);

%% calculate pitch and roll from accel
%pitch
pitch=atan2(-accel(:,1),sqrt(accel(:,2).^2+accel(:,3).^2));

%roll
roll=atan2(accel(:,2),accel(:,3));

%% rotate mag to earth coordinate frame
% (quaternion rotation is used to avoid 'for' loop)

%construct attitude quaternion
qAtt(:,1)=cos(pitch./2).*cos(roll./2);
qAtt(:,2)=cos(pitch./2).*sin(roll./2);
qAtt(:,3)=sin(pitch./2).*cos(roll./2);
qAtt(:,4)=-sin(pitch./2).*sin(roll./2);

%rotate mag to earth frame
M=qM(qAtt,qM([zeros(n,1),mag],[qAtt(:,1),-qAtt(:,2:4)]));
M(:,1)=[];

%% calculate yaw
yaw=atan2(-M(:,2),M(:,1));

%% convert ypr to quaternion
q(:,1)=cos(yaw./2).*cos(pitch./2).*cos(roll./2)+sin(yaw./2).*sin(pitch./2).*sin(roll./2);
q(:,2)=cos(yaw./2).*cos(pitch./2).*sin(roll./2)-sin(yaw./2).*sin(pitch./2).*cos(roll./2);
q(:,3)=cos(yaw./2).*sin(pitch./2).*cos(roll./2)+sin(yaw./2).*cos(pitch./2).*sin(roll./2);
q(:,4)=sin(yaw./2).*cos(pitch./2).*cos(roll./2)-cos(yaw./2).*sin(pitch./2).*sin(roll./2);
end

%% quaternion multiplication function
function [ qOut ] = qM( q1, q2 )
qOut=[q1(:,1).*q2(:,1)-q1(:,2).*q2(:,2)-q1(:,3).*q2(:,3)-q1(:,4).*q2(:,4),... 
      q1(:,1).*q2(:,2)+q1(:,2).*q2(:,1)+q1(:,3).*q2(:,4)-q1(:,4).*q2(:,3),...
      q1(:,1).*q2(:,3)-q1(:,2).*q2(:,4)+q1(:,3).*q2(:,1)+q1(:,4).*q2(:,2),...
      q1(:,1).*q2(:,4)+q1(:,2).*q2(:,3)-q1(:,3).*q2(:,2)+q1(:,4).*q2(:,1)]; 
end

