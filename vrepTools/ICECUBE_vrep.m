% ICECUBE_vrep
% Initialize the V-REP server connection and acquire the handles.
% Run it at the beginning anytime you use the toolbox.
% Do not run 'vrepRemApi_init.m' if you use this file.

% Haopeng Hu
% 2018.05.29

% ICECUEB Communication Protocol v2.0

% vrep: the vrep object
% clientID: the id of the MATLAB applicaiton
% handels: the set of handles in V-REP scene

disp('Connecting to V-REP service...');
vrep = remApi('remoteApi');     % Use remoteApiProto.m
vrep.simxFinish(-1);            % Stop other connections

clientID = vrep.simxStart('127.0.0.1',19997,true,true,6000,5);  % Connect to V-REP
% Attention! port number '19997' is not trivial. It's for the continuous
% operation mode. See remoteApiConnections.txt for details.
% If you want temporary operation mode, change the port number '19997' to 
% a larger one.

if clientID > -1    % Connected!
    disp('V-REP service is connected!');
    handles = struct('ID',clientID);
    vrepPrint(vrep,clientID,'ICECUBE v2.1 (MATLAB) is connected!');
else
    disp('Failed connecting to remote API server');
    vrep.delete();
    return;
end

vrepGetUR5Handles;
vrepGetRG2Handles;

% disp('vrep: the vrep object.');
% disp('clientID: the ID of MATLAB client');
% disp('handles: the handles of objects in V-REP scene.');
