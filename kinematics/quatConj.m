function [qConj]=quatConj(quat)
%GYROALIGN returns the quaternion Conjugate
%q1 is real 

qConj(:,1) = quat(:,1);
qConj(:,2) = -quat(:,2);
qConj(:,3) = -quat(:,3);
qConj(:,4) = -quat(:,4);

end
