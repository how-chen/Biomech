function [X,Y]=AXYB(qA,qB)
% Solves the problem AX=YB
% using the formulation of
%
% Simultaneous robot-world and hand-eye calibration
% F. Dornaika and R. Horaud
%
% Mili Shah
% July 2014
%
% Modified By Howard Chen
% July 2016

n=length(qA);

%Building quaternion representation for A and B
C = zeros(4,4);
for i = 1:n
    QA=[qA(i,1) -qA(i,2) -qA(i,3) -qA(i,4);...
        qA(i,2)  qA(i,1) -qA(i,4)  qA(i,3);...
        qA(i,3)  qA(i,4)  qA(i,1) -qA(i,2);...
        qA(i,4) -qA(i,3)  qA(i,2)  qA(i,1)];    
    
    WB=[qB(i,1) -qB(i,2) -qB(i,3) -qB(i,4);...
        qB(i,2)  qB(i,1)  qB(i,4) -qB(i,3);...
        qB(i,3) -qB(i,4)  qB(i,1)  qB(i,2);...
        qB(i,4)  qB(i,3) -qB(i,2)  qB(i,1)];
    
    C = C - QA'*WB;
end

[u,s,v]=svd(C);
[val,in] = min(n-diag(s));
X = u(:,in)';
Y = v(:,in)';

end
