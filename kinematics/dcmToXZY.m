function [out]=dcmToXZY(dcm)
%%Out(1,2,3)=RxRzRy
ln=length(dcm(1,1,:));
out=zeros(ln,3);
for i=1:1:ln
    out(i,3)=atan2(dcm(1,3,i),dcm(1,1,i));
    out(i,2)=-asin(dcm(1,2,i));
    out(i,1)=atan2(dcm(3,2,i),dcm(2,2,i));
end

