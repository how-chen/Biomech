function [ out ]= EKF(gyro,accel,mag,freq,gyroNoise,accelNoise,magNoise,ca,cb)
%Sampling Period
dT=1/freq;

%Variable Initialization
A=eye(10);
P=zeros(10,10);
W=zeros(10,9);
H=zeros(6,10);

%Process Covariance Matrix
Q=eye(9);
Q(1:3,1:3)=gyroNoise^2.*eye(3);
Q(4:6,4:6)=gyroBias^2.*eye(3);
Q(7:9,7:9)=eye(3);

%Measurement Covariance Matrix
R=eye(6);
R(1:3,1:3)=accelNoise.*eye(3);
R(4:6,4:6)=magNoise.*eye(3);

%Variable constants
g=[ 0 0 9.81];
H(1:3,8:10)=eye(3);
A(8:10,8:10)=ca.*eye(3);
W(5:7,4:6)= dT*eye(3);
W(8:10,7:9)=cb*eye(3);

x=[1 0 0 0 0 0 0 0 0 0]';
out=zeros(length(accel),10);

for i=1:1:length(accel);
    
    %Prediction 
    %quaternions
    x(1)=x(1)+0.5*(-x(2)*(gyro(i,1)-x(5)) - x(3)*(gyro(i,2)-x(6)) - x(4)*(gyro(i,3)-x(7)))*dT;
    x(2)=x(2)+0.5*( x(1)*(gyro(i,1)-x(5)) - x(4)*(gyro(i,2)-x(6)) + x(3)*(gyro(i,3)-x(7)))*dT;
    x(3)=x(3)+0.5*( x(4)*(gyro(i,1)-x(5)) + x(1)*(gyro(i,2)-x(6)) - x(2)*(gyro(i,3)-x(7)))*dT;
    x(4)=x(4)+0.5*(-x(3)*(gyro(i,1)-x(5)) + x(2)*(gyro(i,2)-x(6)) + x(1)*(gyro(i,3)-x(7)))*dT;

    %gyroscope bias
    x(5:7)=x(5:7);

    %acceleration (body frame)
    x(8:10)= ca.*x(8:10);
    
    %normalize quaternion
    n=sqrt(x(1).^2+x(2).^2+x(3).^2+x(4).^2);
    x(1:4)=x(1:4)./n;

    %Partial Derivatives for EKF
    A(1,1)=1;
    A(1,2)=-0.5*(gyro(i,1)-x(5))*dT;
    A(1,3)=-0.5*(gyro(i,2)-x(6))*dT;
    A(1,4)=-0.5*(gyro(i,3)-x(7))*dT;
    A(1,5)= 0.5*x(2)*dT;
    A(1,6)= 0.5*x(3)*dT;
    A(1,7)= 0.5*x(4)*dT;

    A(2,1)= 0.5*(gyro(i,1)-x(5))*dT;
    A(2,2)= 1;
    A(2,3)= 0.5*(gyro(i,3)-x(7))*dT;
    A(2,4)=-0.5*(gyro(i,2)-x(6))*dT;
    A(2,5)=-0.5*x(1)*dT;
    A(2,6)= 0.5*x(4)*dT;
    A(2,7)=-0.5*x(3)*dT;

    A(3,1)= 0.5*(gyro(i,2)-x(6))*dT;
    A(3,2)=-0.5*(gyro(i,3)-x(7))*dT;
    A(3,3)= 1;
    A(3,4)= 0.5*(gyro(i,1)-x(5))*dT;
    A(3,5)=-0.5*x(4)*dT;
    A(3,6)=-0.5*x(1)*dT;
    A(3,7)= 0.5*x(2)*dT;

    A(4,1)= 0.5*(gyro(i,3)-x(7))*dT;
    A(4,2)= 0.5*(gyro(i,2)-x(6))*dT;
    A(4,3)=-0.5*(gyro(i,1)-x(5))*dT;
    A(4,4)= 1;
    A(4,5)= 0.5*x(3)*dT;
    A(4,6)=-0.5*x(2)*dT;
    A(4,7)=-0.5*x(1)*dT;

    W(1,1)=-0.5*x(2)*dT;
    W(1,2)=-0.5*x(3)*dT;
    W(1,3)=-0.5*x(4)*dT;
    W(2,1)= 0.5*x(1)*dT;
    W(2,2)=-0.5*x(4)*dT;
    W(2,3)= 0.5*x(3)*dT;
    W(3,1)= 0.5*x(4)*dT;
    W(3,2)= 0.5*x(1)*dT;
    W(3,3)=-0.5*x(2)*dT;
    W(4,1)=-0.5*x(3)*dT;
    W(4,2)= 0.5*x(2)*dT;
    W(4,3)= 0.5*x(1)*dT;

    %quaternion to DCM conversion
    DCM(1,1)=x(1).^2+x(2).^2-x(3).^2-x(4).^2;
    DCM(1,2)=-2.*x(1).*x(4)+2.*x(2).*x(3);
    DCM(1,3)=2.*x(1).*x(3)+2.*x(2).*x(4);
    DCM(2,1)=2.*x(1).*x(4)+2.*x(2).*x(3);
    DCM(2,2)=x(1).^2-x(2).^2+x(3).^2-x(4).^2;
    DCM(2,3)=-2.*x(1).*x(2)+2.*x(3).*x(4);
    DCM(3,1)=-2.*x(1).*x(3)+2.*x(2).*x(4);
    DCM(3,2)=2.*x(1).*x(2)+2.*x(3).*x(4);
    DCM(3,3)=x(1).^2-x(2).^2-x(3).^2+x(4).^2;
    
    % calculate mag in earth frame
    b=DCM*[mag(i,1);mag(i,2);mag(i,3)];
    b=[sqrt(b(1)^2+b(2)^2),0,b(3)];

    % Measurement Partial Derivatives
    H(1,1) = -2*x(3)*g(3);
    H(1,2) =  2*x(4)*g(3);
    H(1,3) = -2*x(1)*g(3);
    H(1,4) =  2*x(2)*g(3);
    H(2,1) =  2*x(2)*g(3);
    H(2,2) =  2*x(1)*g(3);
    H(2,3) =  2*x(4)*g(3);
    H(2,4) =  2*x(3)*g(3);
    H(3,1) =  2*x(1)*g(3);
    H(3,2) = -2*x(2)*g(3);
    H(3,3) = -2*x(3)*g(3);
    H(3,4) =  2*x(4)*g(3);

    H(4,1) =  2*x(1)*b(1) - 2*x(3)*b(3);
    H(4,2) =  2*x(2)*b(1) + 2*x(4)*b(3);
    H(4,3) = -2*x(3)*b(1) - 2*x(1)*b(3);
    H(4,4) = -2*x(4)*b(1) + 2*x(2)*b(3);
    H(5,1) = -2*x(4)*b(1) + 2*x(2)*b(3); 
    H(5,2) =  2*x(3)*b(1) + 2*x(1)*b(3);
    H(5,3) =  2*x(2)*b(1) + 2*x(4)*b(3);
    H(5,4) = -2*x(1)*b(1) + 2*x(3)*b(3);
    H(6,1) =  2*x(3)*b(1) + 2*x(1)*b(3); 
    H(6,2) =  2*x(4)*b(1) - 2*x(2)*b(3);
    H(6,3) =  2*x(1)*b(1) - 2*x(3)*b(3); 
    H(6,4) =  2*x(2)*b(1) + 2*x(4)*b(3);

    %Kalman Update
    P=A*P*A'+W*Q*W';
    K=P*H'*(H*P*H'+R)^-1;
    x=x+K*([accel(i,1);accel(i,2);accel(i,3);mag(i,1);mag(i,2);mag(i,3)]-[x(8:10)+DCM'*g';DCM'*b']);
    P=(eye(10)-K*H)*P;

    %normalize quaternion
    n=sqrt(x(1).^2+x(2).^2+x(3).^2+x(4).^2);
    x(1:4)=x(1:4)./n;
    out(:,i)=x';
end

