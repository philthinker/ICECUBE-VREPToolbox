function [ forces, torques, state ] = ur5ReadForceSensor( vrep, clientID, handles, N, sampleInterval )
%ur5ReadForceSensor Read the data from UR5's connector
% N: the number of datasets (default:1)
% sampleInterval: the sample interval (dafulat:0.05)
% state: 0 for active, 1 for broken
% forces: 3 x N, values representing the force vector
% torques: 3 x N, values representing the torque vector

if nargin < 5
    sampleInterval = 0.05;
    if nargin < 4
        N = 1;
    end
end

forces = zeros(N,3);
torques = zeros(N,3);

% Start the stream
[~, state] = vrep.simxReadForceSensor(clientID,handles.ur5connector,vrep.simx_opmode_streaming);

if state == 0
    % the data is available
%     disp('Available!');
    i = 1; k = 1;
    while i<=N && k<= 40000
        [res,~,forceVec,torqueVec] = vrep.simxReadForceSensor(clientID,handles.ur5connector,vrep.simx_opmode_buffer);
        if res == vrep.simx_return_ok
            % the sensor is working
%             disp('Get a dataset');
            forces(i,:) = forceVec;
            torques(i,:) = torqueVec;
            i = i + 1;
        end
        k = k + 1;
        pause(sampleInterval);
    end
end

% Stop the stream
[~,state] = vrep.simxReadForceSensor(clientID,handles.ur5connector,vrep.simx_opmode_discontinue);

end

