function [  ] = ur5RMLMoveToJointPositions( icecube, targetPositions )
%ur5RMLMoveToJointPositions Reflexes Motion Library - Joint space
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPositions: 1 x 6 vector, the target joint positions

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.07.05

icecube.vrep.simxPauseCommunication(icecube.clientID, 1);
icecube.vrep.simxSetIntegerSignal(icecube.clientID, 'ICECUBE_0', 11, icecube.vrep.simx_opmode_oneshot);
for i = 1:6
    icecube.vrep.simxSetFloatSignal(icecube.clientID, strcat('ICECUBE_',int2str(i)), targetPositions(i), icecube.vrep.simx_opmode_oneshot);
end
icecube.vrep.simxPauseCommunication(icecube.clientID, 0);

icecube.wait();

%% ICECUBE 2.2
% % [  ] = ur5RMLMoveToJointPositions( vrep, clientID, targetPositions )
% vrep.simxPauseCommunication(clientID, 1);
% vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 11, vrep.simx_opmode_oneshot);
% for i = 1:6
%     vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)), targetPositions(i), vrep.simx_opmode_oneshot);
% end
% vrep.simxPauseCommunication(clientID, 0);
% 
% ICECUBE_wait(vrep,clientID,0.2,1000);


end

