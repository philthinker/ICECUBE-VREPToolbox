function [  ] = ur5MoveToJointPosition( icecube, targetPositions )
%ur5MoveToJointPosition Reflexes Motion Library - Joint space
% icecube: the ICECUBE object
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

end

