function [ tipPosition ] = ur5GetIKTipPosition( vrep, clientID, handles )
%ur5GetIKTipPosition Get UR5's ikTip's position from the V-REP scene
% vrep: the vrep object
% clientID: the MATLAB client ID
% handles: the handles of V-REP scence
% tipPosition: the cartesian position of UR5's ikTip

[res, tipPosition] = vrep.simxGetObjectPosition(clientID,handles.ur5ikTip,-1,vrep.simx_opmode_blocking);
vrchk(vrep,res);

end

