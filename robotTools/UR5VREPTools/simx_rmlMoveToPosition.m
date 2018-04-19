function [ res ] = simx_rmlMoveToPosition( vrep, clientID, simx_opmode, targetPos, targetQua )
%simx_rmlMoveToPosition Move by ik group
% vrep: the vrep object
% clientID: the MATLAB client ID
% simx_opmode: the operation mode
% targetPos: 1 x 3 vector, the target xyz position
% targetQua: 1 x 4 vector, the target quaternion
% res: the return code, you can use 'vrepTools\vrchk' to check it

% 'Lua\UR5IKRemote_v1.lua' is required

[res,resInts, resFloats, resStrings, resBuffer] = vrep.simxCallScriptFunction(clientID, 'UR5', ...
    vrep.sim_scripttype_childscript, 'rem_rmlMoveToPosition', [], [targetPos,targetQua], [], [], simx_opmode);

if simx_opmode == vrep.simx_opmode_oneshot
    res = vrep.simx_error_noerror;
end

end

% Refer to : http://www.coppeliarobotics.com/helpFiles/en/remoteApiExtension.htm
% for more details