function [ data ] = importActigraph(PathName,FileName)
%ACTIGRAPHTOIMU Imports Actigraph Link Files 
%Author: Howard Chen
%Date: 3-29-17

datafile=csvread(strcat(PathName,FileName),11,1);

%Accelerometer
data(:,1:3)=datafile(:,1:3).*9.81;

%Gyroscope
data(:,4:6)=degtorad(datafile(:,5:7));

%Magnetometer (May need to re-order it)
data(:,7:9)=datafile(:,8:10);
   
end

