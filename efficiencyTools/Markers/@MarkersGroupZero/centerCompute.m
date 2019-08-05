function [XYZ] = centerCompute(obj)
%centersCompute Compute the positions of the MarkersGroup's center.
%   You can rewrite the function in the subclass for a better consideration of the center.
%   @MarkersGroupZero

%% Taking the Marker 2 as the center

if obj.M >= 2
    XYZ = obj.Markers(2).XYZ;
else
    XYZ = obj.Markers(1).XYZ;
end

end

