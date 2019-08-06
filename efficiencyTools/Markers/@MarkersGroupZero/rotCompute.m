function [SO3] = rotCompute(obj)
%rotCompute Compute the rotation matrix of each frame based on the posture
%of the first frame. Note that only THREE Markers are considerred which
%means that M >= 3.
%   SO3: 3 x 3 x N, the rotation matrices
%   @ MarkersGroupZero

SO3 = repmat(eye(3),[1,1,obj.N]);

end

