% ICECUBE_init
% Initialize the V-REP server connection and acquire the handles.
% Run it at the beginning anytime you use the toolbox.

% Haopeng Hu
% 2018.06.25

% ICECUEB Communication Protocol v2.0

% vrep: the vrep object
% clientID: the id of the MATLAB applicaiton
% handles: the set of handles in V-REP scene

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
    vrepPrint(vrep,clientID,'A MATLAB client is found (ICECUBE v2.2).');
    handles = struct('ID',clientID);
else
    disp('Failed connecting to remote API server');
    vrep.delete();
    return;
end

% disp('vrep: the vrep object.');
% disp('clientID: the ID of MATLAB client');
% disp('handles: the handles of objects in V-REP scene.');