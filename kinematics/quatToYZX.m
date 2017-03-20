function [ out ] = quatToYZX( q )
%QUATTOYXZ Summary of this function goes here
%   Detailed explanation goes here

    out(:,1)=-atan2(-2.*q(:,1).*q(:,3)+2.*q(:,2).*q(:,4),q(:,1).^2+q(:,2).^2-q(:,3).^2-q(:,4).^2);
    out(:,2)=asin(2.*q(:,1).*q(:,4)+2.*q(:,2).*q(:,3));
    out(:,3)=-atan2(-2.*q(:,1).*q(:,2)+2.*q(:,3).*q(:,4),q(:,1).^2-q(:,2).^2+q(:,3).^2-q(:,4).^2);

end

