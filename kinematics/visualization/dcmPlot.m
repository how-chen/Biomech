function dcmPlot(a)
% dcmPlot  will plot the coordinate axis of a 3x3 rotation matrix
% For matlab 2016A x-axis: blue, y-axis: red, z-axis: yellow
% Author: Howard Chen
% Date: 3/20/16
% Update: 3/17/16
% 

plot3([0 a(1,1)],[0 a(2,1)],[0 a(3,1)],...
      [0 a(1,2)],[0 a(2,2)],[0 a(3,2)],...
      [0 a(1,3)],[0 a(2,3)],[0 a(3,3)],'linewidth',2);
axis([-2 2 -2 2 -2 2]);
