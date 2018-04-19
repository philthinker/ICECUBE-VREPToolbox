function [  ] = sim_rmlMoveToJointPositions( vrep, clientID, targetPos )
%sim_rmlMoveToJointPositions Reflexxes Motion Library - Joint space
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPos: 1 x 6 vector, the target joint positions

% 'Lua\UR5IKRemote_v2.lua' is required


vrep.simxPauseCommunication(clientID, 1);
vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 1, vrep.simx_opmode_oneshot);
for i = 1:6
    vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetPos(i), vrep.simx_opmode_oneshot);
end
vrep.simxPauseCommunication(clientID, 0);


end

