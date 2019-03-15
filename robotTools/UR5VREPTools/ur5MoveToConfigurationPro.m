function [ joints ] = ur5MoveToConfigurationPro( icecube, targetPosition, targetQuaternion )
%ur5MoveToConfigurationPro Reflexxes Motion Library - Move by ik group
% icecube: the icecube object
% targetPosition: 1 x 3 vector, the target xyz position
% targetQuaternion: 1 x 4 vector, the target quaternion
% joints: M x 6, the route joint positions
% If you DO NOT need route joint positions, use 'ur5MoveToConfiguration.m'
% instead for more efficiency.

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.07.05

% %%%% DEPRECATED %%%%

targetQuaternion = toVREPQuat(targetQuaternion);
joints = zeros(icecube.TIMEOUT,6);

%% Initialize data stream
state = zeros(1,6);
for i = 1:6
    [state(i),~] = icecube.vrep.simxGetJointPosition(icecube.clientID, icecube.handles.ur5Joints(i), icecube.vrep.simx_opmode_streaming);
end
% Note that when you start stream, there maybe NO value in the buffer
% available. Then the return code may be simx_return_novalue_flag rather
% than simx_return_ok. 
if all((state == icecube.vrep.simx_return_ok) | (state == icecube.vrep.simx_return_novalue_flag))
    % All joints are available
    endSignal = 1;
else
    % At least one joint is not available
    endSignal = 0;
end
%% Motion Plan
icecube.vrep.simxPauseCommunication(icecube.clientID, 1);
icecube.vrep.simxSetIntegerSignal(icecube.clientID, 'ICECUBE_0', 21, icecube.vrep.simx_opmode_oneshot);
for i = 1:3
    icecube.vrep.simxSetFloatSignal(icecube.clientID, strcat('ICECUBE_',int2str(i)), targetPosition(i), icecube.vrep.simx_opmode_oneshot);
end
for i = 4:7
    icecube.vrep.simxSetFloatSignal(icecube.clientID, strcat('ICECUBE_',int2str(i)), targetQuaternion(i-3), icecube.vrep.simx_opmode_oneshot);
end
icecube.vrep.simxPauseCommunication(icecube.clientID, 0);
%% Read the joints and wait for the ending signal
j=1;
while j <= icecube.TIMEOUT && endSignal ~= 0
    % Read the joints
    for k = 1:6
        [res, jointPosition] = icecube.vrep.simxGetJointPosition(icecube.clientID, icecube.handles.ur5Joints(k), icecube.vrep.simx_opmode_buffer);
        if res == icecube.vrep.simx_return_ok
            % A good value is accessed from the buffer
            joints(j,k) = jointPosition;
        elseif res == icecube.vrep.simx_return_novalue_flag
            % No value in the buffer available currently
            joints(j,k) = NaN;
        else
            % An error occurred
            joints(j,k) = NaN;
        end
    end
    pause(icecube.step);
    % Wait for the end signal
    [res, endSignal] = icecube.vrep.simxGetIntegerSignal(icecube.clientID, 'ICECUBE_0', icecube.vrep.simx_opmode_blocking);
    icecube.vrchk(res);
    pause(icecube.step);
    j = j+1;
end

%% Never forget to stop the streams
for i=1:6
    [~,~] = icecube.vrep.simxGetJointPosition(icecube.clientID,icecube.handles.ur5Joints(i),icecube.vrep.simx_opmode_discontinue);
end

%% Get rid of the last zeros
if endSignal == 0
    % Stop normally
    joints = joints(1:j-1,:);
end

%% Get rid of the first NaNs

% 'efficiencyTools\UR5Tools' is required
joints = ur5JointTrajMend(joints);

end

