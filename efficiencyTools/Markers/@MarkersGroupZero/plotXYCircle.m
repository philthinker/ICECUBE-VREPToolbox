function [] = plotXYCircle(obj,center,radius,color)
%plotXYCircle Plot a circle in XY plane
%   center: 1 x 2 or 1 x 3, the center
%   radius: the radius
%   color: 'r' or 'b' ..., the color

%   @MarkersGroupZero

if size(center,2) == 2
    rectangle('Position',[center-radius,radius,radius],'Curvature',[1,1],'EdgeColor',color);
elseif size(center,2) == 3
    t = (0:0.01:2*pi);
    circleX = center(1) + sin(t)*radius;
    circleY = center(2) + cos(t)*radius;
    circleZ = center(3) * ones(size(t));
    plot3(circleX,circleY,circleZ,color);
end
end

