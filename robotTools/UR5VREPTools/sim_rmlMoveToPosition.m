function [  ] = sim_rmlMoveToPosition( vrep, clientID, targetPos, targetQua )
%sim_rmlMoveToPosition Move by ik group
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPos: 1 x 3 vector, the target xyz position
% targetQua: 1 x 4 vector, the target quaternion

% 'Lua\UR5IKRemote_v2.lua' is required

vrep.simxPauseCommunication(clientID, 1);
vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 2, vrep.simx_opmode_oneshot);
for i = 1:3
    vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetPos(i), vrep.simx_opmode_oneshot);
end
for i = 4:7
    vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetQua(i-3), vrep.simx_opmode_oneshot);
end
vrep.simxPauseCommunication(clientID, 0);

end

