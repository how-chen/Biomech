function [omcQuat]= importOptitrack(PathName,FileName,colStart)
datafile=dlmread(strcat(PathName,FileName),',',8,0); 
quat=datafile(:,colStart:colStart+3); 

%converts Optitrack quaternions to IMU quaternions
omcQuat=[quat(:,4),quat(:,3),quat(:,1),quat(:,2)];