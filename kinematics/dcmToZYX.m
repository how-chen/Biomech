function [out]=dcmToZYX(dcm)
%%Out=RzRxRy

ln=length(dcm(1,1,:));
out=zeros(ln,3);
for i=1:1:ln
    %z
    out(i,1)=atan2(dcm(2,1,i),dcm(1,1,i));
    %y
    out(i,2)=-asin(dcm(3,1,i));
    %x
    out(i,3)=atan2(dcm(3,2,i),dcm(3,3,i));
end
