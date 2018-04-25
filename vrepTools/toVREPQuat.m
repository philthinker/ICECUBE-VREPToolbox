function [ VREPQuat ] = toVREPQuat( sxyzQuat )
%toVREPQuat Transform a quaternion of <s,x,y,z> to VREP form (<<x,y,z,s>>
% sxyzQuat: the Peter Corke's UnitQuaternion class
% VREPQuat: the quaternion <x,y,z,s>

% Peter Corke's Robotics Toolbox is required.

VREPQuat = zeros(1,4);
temp = sxyzQuat.double();
VREPQuat(4) = temp(1);
VREPQuat(1) = temp(2);
VREPQuat(2) = temp(3);
VREPQuat(3) = temp(4);

end

