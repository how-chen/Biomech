function [ output_args ] = coord(a,l)
%PLOTVEC Summary of this function goes here
%   Detailed explanation goes here
a=a*[l,0,0,0;
    0,l,0,0;
    0,0,l,0;
    0,0,0,1];

plot3([a(1,4),a(1,4)+a(1,1)],[a(2,4),a(2,4)+a(2,1)],[a(3,4),a(3,4)+a(3,1)],...
    [a(1,4),a(1,4)+a(1,2)],[a(2,4),a(2,4)+a(2,2)],[a(3,4),a(3,4)+a(3,2)],...
    [a(1,4),a(1,4)+a(1,3)],[a(2,4),a(2,4)+a(2,3)],[a(3,4),a(3,4)+a(3,3)],'linewidth',2)
axis([-5 5 -5 5 -5 5])
end

