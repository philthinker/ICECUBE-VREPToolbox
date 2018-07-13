function [ tipQuat ] = ur5GetIKTipQuaternion( icecube )
%ur5GetIKTipQuaternion Get UR5's ikTip's quaternion from the V-REP scene
% icecube: the icecube object
% tipQuat: the quaternion of UR5's ikTip

[res, tipQuat] = icecube.vrep.simxGetObjectQuaternion(icecube.clientID, icecube.handles.ur5ikTip, -1, icecube.vrep.simx_opmode_blocking);
icecube.vrchk(res);
tipQuat = toMATLABQuat(tipQuat);

end

