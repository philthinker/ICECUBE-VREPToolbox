function [ VREPQuat ] = toVREPQuat( unitQuat )
%toVREPQuat Transform a quaternion of <s,x,y,z> to VREP form (x,y,z,s)
% sxyzQuat: the <w,x,y,z> quaternion
% VREPQuat: the quaternion <x,y,z,s>

% Peter Corke's Robotics Toolbox is required.

VREPQuat = zeros(1,4);
VREPQuat(4) = unitQuat(1);
VREPQuat(1) = unitQuat(2);
VREPQuat(2) = unitQuat(3);
VREPQuat(3) = unitQuat(4);

end

