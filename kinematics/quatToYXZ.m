function [ out ] = quatToYXZ( quat )
out(:,1)=atan2(2.*(quat(:,1).*quat(:,3)+quat(:,2).*quat(:,4)),quat(:,1).^2-quat(:,2).^2-quat(:,3).^2+quat(:,4).^2);
out(:,2)=-asin(2.*(quat(:,3).*quat(:,4)-quat(:,1).*quat(:,2)));
out(:,3)=atan2(2.*(quat(:,2).*quat(:,3)+quat(:,1).*quat(:,4)),quat(:,1).^2-quat(:,2).^2+quat(:,3).^2-quat(:,4).^2);


end
