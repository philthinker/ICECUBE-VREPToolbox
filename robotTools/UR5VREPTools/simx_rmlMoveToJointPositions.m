function [ res ] = simx_rmlMoveToJointPositions( vrep, clientID, simx_opmode, targetPos )
%simx_rmlMoveToJointPositions Reflexxes Motion Library - Joint space
% vrep: the vrep object
% clientID: the MATLAB client ID
% simx_opmode: the operation mode
% targetPos: 1 x 6 vector, the target joint positions
% res: the return code, you can use 'vrepTools\vrchk' to check it

% 'Lua\UR5IKRemote.lua' is required

[res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(clientID, 'UR5',...
    vrep.sim_scripttype_childscript, 'rem_rmlMoveToJointPositions',[],targetPos,[],[],simx_opmode);

if simx_opmode == vrep.simx_opmode_oneshot
    res = vrep.simx_error_noerror;
end

end

% Refer to : http://www.coppeliarobotics.com/helpFiles/en/remoteApiExtension.htm
% for more details