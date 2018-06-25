function [ sxyzQuat ] = toMATLABQuat( vrepQuat )
%toMATLABQuat Transform a quaternion of VREP form (x,y,z,s) to <s,x,y,z>
% wxyzQuat: the <s,x,y,z> quaternion
% VREPQuat: the quaternion <x,y,z,s>

sxyzQuat = zeros(1,4);
sxyzQuat(1) = vrepQuat(4);
sxyzQuat(2) = vrepQuat(1);
sxyzQuat(3) = vrepQuat(2);
sxyzQuat(4) = vrepQuat(3);

end

