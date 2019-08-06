function [SO3,p] = threeMarkerPos(obj,queryMarkers,originMarkers)
%threeMarkerPos Compute the relative posture of the query markers according
%to the origin markers. At least three markers are required.
%   %%%%------------------------------------------------------%%%%
%   By default, the middle marker is taken as the origin and the center of the left two
%   markers is used to indicate the orientation. This can be modified for
%   future use.
%   %%%%------------------------------------------------------%%%%
%   queryMarkers: 3 x 3, xyz of the query markers
%   originMarkers: 3 x 3, xyz of the origin markers (optional)
%   SO3: 3 x 3, SO(3) rotation matrix
%   p: 3 x 1, position vector
%   @MarkersGroupZero

if nargin < 3
    % Compute the absolute position and orientation of the query markers
    originMarkers = repmat([0,0,0],[3,1]);
end

%%%%--------------------------------------------------%%%%
% Modify the following codes if you want other origin defination
origin = originMarkers(2,:);
originOri = (originMarkers(1,:) + originMarkers(3,:))/2 - origin;
query = queryMarkers(2,:);
queryOri = (queryMarkers(1,:) + queryMarkers(3,:))/2 - query;
%%%%--------------------------------------------------%%%%

SO3 = eye(3);
p = query - origin;

end

