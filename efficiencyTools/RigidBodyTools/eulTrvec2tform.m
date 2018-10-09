function [ tformOut ] = eulTrvec2tform( eulIn, vecIn )
%eulVec2tform Transform euler angles together with trasfer vector to tform
% eulIn: N x [eulz,euly,eulx], MATLAB definition (ZYX relative reference frame)
% vecIn: N x [x,y,z]
% tformOut: 4 x 4 x N, homogeneous transformations

tformOut = eul2tform(eulIn);
tformOut(1,4,:) = vecIn(:,1);
tformOut(2,4,:) = vecIn(:,2);
tformOut(3,4,:) = vecIn(:,3);

end

