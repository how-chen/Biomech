function [q2] = gyroAlign( omcQuat,imuGyro,freq )
%GYROALIGN aligns the OMC Local Frame to the IMU Local Frame
% 
% LF alignment is conducted following: 
% De Vries, W. H. K., Veeger, H. E. J., Baten, C. T. M., & 
% Van Der Helm, F. C. T. (2009). Magnetic distortion in motion 
% labs, implications for validating inertial magnetic sensors. 
% Gait & posture, 29(4), 535-541.
%
% Angular rate measurements from OMC is calculated following:
% Brodie, M. A., Walmsley, A., & Page, W. (2008). 
% Dynamic accuracy of inertial measurement units during simple
% pendulum motion: Technical Note. Computer methods in biomechanics 
% and biomedical engineering, 11(3), 235-242.
%
% Input: omcQuat- quaternion from OMC
%        imuGyro- angular rate measurements (rad/s) from IMU gyroscope
%        freq- common sampling frequency

% Author: Howard Chen
% Date: 3/6/2017


% Low-pass filter to remove noise
filtCutoff=5;
[lowpass_b,lowpass_a]=butter(2,filtCutoff/(freq/2),'low'); 

imuGyro=filtfilt(lowpass_b,lowpass_a,imuGyro(2:end,:)); 
omcQuat=filtfilt(lowpass_b,lowpass_a,omcQuat); 

% Get angular rate measurements from OMC quaternion 
sz=size(omcQuat); 
omcOmega=zeros(sz(1)-1,3);

for i=1:1:length(omcQuat)-1
    %calculate dQ
    dQ=quatMultiply(quatConj(omcQuat(i,:)),omcQuat(i+1,:));
    dQ=quatNormalize(dQ);
    
    %calculate theta and angle
    dTheta=2*acos(dQ(1));
    Ux=dQ(2)/sin(dTheta/2);
    Uy=dQ(3)/sin(dTheta/2);
    Uz=dQ(4)/sin(dTheta/2);
    
    %calculate Omega
    omegaNorm=freq*dTheta;
    omcOmega(i,1)=omegaNorm*Ux;
    omcOmega(i,2)=omegaNorm*Uy;
    omcOmega(i,3)=omegaNorm*Uz;
end

% calculate LF rotation matrix
R=imuGyro'*pinv(omcOmega');

%orthogonize matrix
[U,W,V]=svd(R');
R = U*V'; 

%Convert to Quaternion
q2 = zeros(1,4); 
q2(1)=0.5.*sqrt(1+R(1,1)+R(2,2)+R(3,3));
q2(2)=(R(3,2) - R(2,3))./(4.*q2(1));
q2(3)=(R(1,3) - R(3,1))./(4.*q2(1));
q2(4)=(R(2,1) - R(1,2))./(4.*q2(1));

end

