function [  ] = XYZScatter( obj,S,C )
%XYZScatter Scatter the points
%   S: N x 1, sizes (unnecessary)
%   C, N x 1, colors (unnecessary)
%   @SimpleMarker

X = obj.XYZ(:,1);
Y = obj.XYZ(:,2);
Z = obj.XYZ(:,3);

if nargin == 1
    % default S,C
    S = 2*ones(size(X));
    C = repmat([0 0 1],[obj.N,1]);
elseif nargin == 2
    % default C
    C = repmat([0 0 1],[obj.N,1]);
end

figure;

scatter3(X,Y,Z,S,C);
xlabel('x');ylabel('y');zlabel('z');
grid on;    hold on;    axis equal;

end

