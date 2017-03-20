function [loc]= importOptitrackPos(PathName,FileName,colStart)
datafile=dlmread(strcat(PathName,FileName),',',8,0); 
loc2=datafile(:,colStart+4:colStart+7); 

%swap convention
loc(:,1)=-loc2(:,1);
loc(:,3)=loc2(:,2);
loc(:,2)=loc2(:,3);