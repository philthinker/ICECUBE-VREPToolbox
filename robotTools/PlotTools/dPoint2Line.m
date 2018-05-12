function [ distance ] = dPoint2Line( P, L )
%dPoint2Line Compute the distance between a point and a line in 2d/3d space
% point: N x 2/3, the points
% line: 2 x 2/3, two points in the line

N = size(P,1);
distance = zeros(N,1);
for i = 1:N
    distance(i) = norm(cross(L(2,:)-L(1,:),P(i,:)-L(1,:)))/norm(L(2,:)-L(1,:));
end

end

