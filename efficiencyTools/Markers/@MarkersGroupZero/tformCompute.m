function [SE3] = tformCompute(obj)
%tformCompute Compute the homogeneous transformation matrix of each frame
%based on the posture of the first frame. Note that only THREE Markers are
%considerred which means that M >= 3.
%   SE3: 3 x 3 x N, the homogeneous transformation matrices
%   @ MarkersGroupZero

SE3 = repmat(eye(4),[1,1,obj.N]);

end

