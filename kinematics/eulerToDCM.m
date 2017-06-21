function [ R ] = eulerToDCM( eul )
% ZYX sequence
% Author: Howard Chen
% Affiliation: GAV Lab | Mechanical Engineering | Auburn University

R = [cos(eul(1))*cos(eul(2)) cos(eul(1))*sin(eul(2))*sin(eul(3))-cos(eul(3))*sin(eul(1)) sin(eul(1))*sin(eul(3))+cos(eul(1))*cos(eul(3))*sin(eul(2));
    cos(eul(2))*sin(eul(1)) cos(eul(1))*cos(eul(3))+sin(eul(1))*sin(eul(2))*sin(eul(3)) cos(eul(3))*sin(eul(1))*sin(eul(2)) - cos(eul(1))*sin(eul(3))
    -sin(eul(2)) cos(eul(2))*sin(eul(3)) cos(eul(2))*cos(eul(3))]; 
    
end

