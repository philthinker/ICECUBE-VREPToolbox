function [  ] = ur5RMLMoveToPosition( icecube, targetPosition, targetQuaternion )
%ur5RMLMoveToPosition Reflexxes Motion Library - Move by ik group
% icecube: the icecube object
% targetPosition: 1 x 3 vector, the target xyz position
% targetQuaternion: 1 x 4 vector, the target quaternion
% operationMode: 0 for oneshot mode while 1 for blocking mode (not neces.)

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.07.05

targetQuaternion = toVREPQuat(targetQuaternion);
icecube.vrep.simxPauseCommunication(icecube.clientID, 1);
icecube.vrep.simxSetIntegerSignal(icecube.clientID, 'ICECUBE_0', 21, icecube.vrep.simx_opmode_oneshot);
for i = 1:3
    icecube.vrep.simxSetFloatSignal(icecube.clientID, strcat('ICECUBE_',int2str(i)), targetPosition(i), icecube.vrep.simx_opmode_oneshot);
end
for i = 4:7
    icecube.vrep.simxSetFloatSignal(icecube.clientID, strcat('ICECUBE_',int2str(i)), targetQuaternion(i-3), icecube.vrep.simx_opmode_oneshot);
end
icecube.vrep.simxPauseCommunication(icecube.clientID, 0);

icecube.wait()

%% ICECUBE 2.2
% % [  ] = ur5RMLMoveToPosition( vrep, clientID, targetPosition, targetQuaternion )
% targetQuaternion = toVREPQuat(targetQuaternion);
% vrep.simxPauseCommunication(clientID, 1);
% vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 21, vrep.simx_opmode_oneshot);
% for i = 1:3
%     vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetPosition(i), vrep.simx_opmode_oneshot);
% end
% for i = 4:7
%     vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetQuaternion(i-3), vrep.simx_opmode_oneshot);
% end
% vrep.simxPauseCommunication(clientID, 0);
% 
% ICECUBE_wait(vrep,clientID,0.2,1000);


end

