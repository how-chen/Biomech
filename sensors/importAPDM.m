function [data]=importAPDM(PathName,FileName,ID)
    %%imports data from APDM Sensor
    %ID=Sensor ID
    %data(:,1:3)=accelerometer
    %data(:,4:6)=gyro
    %data(:,7:9)=magnetometer
    %data(:,10:13)=quaternion
   
    data(:,1:3)=h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Accelerometers'))';
    data(:,4:6)=h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Gyroscopes'))';
    data(:,7:9)=h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Magnetometers'))';
    data(:,10:13)=h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Orientation'))';
end
