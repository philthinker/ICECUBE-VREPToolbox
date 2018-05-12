function [  ] = xyzPlot( xyzData, buffer, figTitle )
%xyzPlot Plot the x,y,z data in a 3D plot figure.
% xyzData: [x,y,z,...], the data to be ploted.
% buffer: the color (default 'k')
% figTitle: the title (default '')

if nargin < 3
    figTitle = '';
    if nargin < 2
        buffer = 'k';
    end
end

figure();
ax = gca;
ax.Projection = 'orthographic';
plot3(xyzData(:,1),xyzData(:,2),xyzData(:,3),buffer);
grid on
axis equal
xlabel('x');
ylabel('y');
zlabel('z');
title(figTitle);


end

