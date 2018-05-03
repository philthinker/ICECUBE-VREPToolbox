function [  ] = sim_rmlMoveToJointPositions_pro( vrep,clientID,targetPos, motionMode )
%sim_rmlMoveToJointPositions_pro Reflexxes Motion Library - Joint space
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPos: 1 x 6 vector, the target joint positions
% motionMode: 1,2 or 3 stands for starting point, route point or ending
% point respectively

% ICECUBE Communication Protocol v1.2
% 'Lua\UR5IKRemote_v12.lua' is required

vrep.simxPauseCommunication(clientID, 1);
if motionMode == 1
    % Starting Point
    % disp('Starting Point');
    vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 5, vrep.simx_opmode_oneshot);
elseif motionMode == 2
    % Route Point
    % disp('Route Point');
    vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 6, vrep.simx_opmode_oneshot);
elseif motionMode == 3
    % Ending Point
    % disp('Ending Point');
    vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 7, vrep.simx_opmode_oneshot);
end
for i = 1:6
    vrep.simxSetFloatSignal(clientID, strcat('ICECUBE_',int2str(i)),...
        targetPos(i),vrep.simx_opmode_oneshot);
end
vrep.simxPauseCommunication(clientID, 0);

end

