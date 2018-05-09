function [ tipPosition ] = sim_getUR5IKTipPosition( vrep, clientID, tipHandle )
%sim_getUR5IKTipPosition Get the cartesian of UR5's ikTip
% vrep: the vrep object
% clientID: the MATLAB client ID
% tipHandle: the handle of UR5's ikTip
% tipPosition: the cartesian position of UR5's ikTip

[res, tipPosition] = vrep.simxGetObjectPosition(clientID,tipHandle,-1,vrep.simx_opmode_blocking);
vrchk(vrep,res);

end

