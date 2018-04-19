function [ res ] = sim_rmlMoveToPosition( vrep, clientID, simx_opmode, targetPos, targetQua )
%sim_rmlMoveToPosition Move by ik group
% vrep: the vrep object
% clientID: the MATLAB client ID
% simx_opmode: the operation mode
% targetPos: 1 x 3 vector, the target xyz position
% targetQua: 1 x 4 vector, the target quaternion
% res: the return code, you can use 'vrepTools\vrchk' to check it

% 'Lua\UR5IKRemote_v2.lua' is required


end

