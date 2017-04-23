function [loc]= importOptitrackPos(PathName,FileName,colStart)
% imports position data from optitrack
% input: Path, file, and first column of data
% output: loc(nx3) column 1:3 corresponds to x y z
%
% coordinate transformed to align with matlab 
% Date: 03/20/2017
% Author: Howard Chen
datafile=dlmread(strcat(PathName,FileName),',',8,0); 
loc2=datafile(:,colStart+4:colStart+7); 

%swap convention
loc(:,1)=-loc2(:,1);
loc(:,3)=loc2(:,2);
loc(:,2)=loc2(:,3);
