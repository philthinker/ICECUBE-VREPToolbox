function [  ] = ur5RMLMoveToJointPositions( vrep, clientID, targetPositions )
%ur5RMLMoveToJointPositions Reflexxes Motion Library - Joint space
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPos: 1 x 6 vector, the target joint positions
% operationMode: 0 for oneshot mode while 1 for blocking mode (not neces.)

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.05.15


vrep.simxPauseCommunication(clientID, 1);
vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 11, vrep.simx_opmode_oneshot);
for i = 1:6
    vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetPositions(i), vrep.simx_opmode_oneshot);
end
vrep.simxPauseCommunication(clientID, 0);

ICECUBE_wait(vrep,clientID,0.2,1000);


end

