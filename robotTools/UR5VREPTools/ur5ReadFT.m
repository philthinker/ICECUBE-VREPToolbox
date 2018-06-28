function [ FTdata ] = ur5ReadFT( vrep, clientID, handles, N, sampleInterval )
%ur5ReadFT Read the data from UR5's connector as a 6-d vector
% N: the number of datasets (default:1)
% sampleInterval: the sample interval (dafulat:0.05)
% FTdata: N x 6, the sensor signal from UR5's connector

if nargin < 5
    sampleInterval = 0.05;
    if nargin < 4
        N = 1;
    end
end

FTdata = zeros(N,6);

% Start the stream
[~, state] = vrep.simxReadForceSensor(clientID,handles.ur5connector,vrep.simx_opmode_streaming);

if state == 0
    % the data is available
    i = 1; k = 1;
    while i<=N && k<= 40000
        [res,~,forceVec,torqueVec] = vrep.simxReadForceSensor(clientID,handles.ur5connector,vrep.simx_opmode_buffer);
        if res == vrep.simx_return_ok
            % the sensor is working
            FTdata(i,1:3) = forceVec;
            FTdata(i,4:6) = torqueVec;
            i = i + 1;
        end
        k = k + 1;
        pause(sampleInterval);
    end
end

% Stop the stream
[~,~] = vrep.simxReadForceSensor(clientID,handles.ur5connector,vrep.simx_opmode_discontinue);

end

