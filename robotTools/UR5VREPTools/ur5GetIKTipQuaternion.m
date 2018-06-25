function [ tipQuat ] = ur5GetIKTipQuaternion( vrep, clientID, handles )
%ur5GetIKTipQuaternion Get UR5's ikTip's quaternion from the V-REP scene
% vrep: the vrep object
% clientID: the MATLAB client ID
% handles: the handles of V-REP scence
% tipQuat: the quaternion of UR5's ikTip

[res, tipQuat] = vrep.simxGetObjectQuaternion(clientID, handles.ur5ikTip, -1, vrep.simx_opmode_blocking);
vrchk(vrep,res);
tipQuat = toMATLABQuat(tipQuat);

end

