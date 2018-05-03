function [ res ] = sim_rmlMoveToJointPositions_series( vrep,clientID,targetPoses )
%sim_rmlMoveToJointPositions_series Follow a set of route points in Joint
%Space.
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPoses: N x 6 vector, the target joint positions

% ICECUBE Communication Protocol v1.1

% 'UR5_IKRemote_v11.lua' is required

N = size(targetPoses,1);
for j = 1:N-1
    vrep.simxPauseCommunication(clientID, 1);
    vrep.simxSetIntegerSignal(clientID,'ICECUBE_O', 4, vrep.simx_opmode_oneshot);
    for i = 1:6
        vrep.simxSetFloatSignal(clientID,strcat('ICECUBE_',int2str(i)),...
            targetPoses(j,i),vrep.simx_opmode_oneshot);
    end
    vrep.simxPauseCommunication(clientID, 0);
    ICECUBE_wait(vrep,clientID,0.1,1000);
end
vrep.simxPauseCommunication(clientID, 1);
vrep.simxSetIntegerSignal(clientID,'ICECUBE_0',5,vrep.simx_opmode_oneshot);
for i = 1:6
    vrep.simxSetFloatSignal(clientID,strcat('ICECUBE_',int2str(i)),...
        targetPoses(N,i),vrep.simx_opmode_oneshot);
end
vrep.simxPauseCommunication(clientID,0);
disp('Motion cmd has been sent');
end

