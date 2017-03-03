function [aa] = dcmToQuat(DCM)
ln=length(DCM(1,1,:));
aa=zeros(ln,4);
for i=1:1:ln
    aa(i,1)=0.5.*sqrt(1+DCM(1,1,i)+DCM(2,2,i)+DCM(3,3,i));
    aa(i,2)=(DCM(3,2,i)-DCM(2,3,i))./(4.*aa(i,1));
    aa(i,3)=(DCM(1,3,i)-DCM(3,1,i))./(4.*aa(i,1));
    aa(i,4)=(DCM(2,1,i)-DCM(1,2,i))./(4.*aa(i,1));
end


