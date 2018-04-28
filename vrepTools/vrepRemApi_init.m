% vrepRemApi_init
% Initialize the V-REP server connection
% Run it at the beginning anytime you use the toolbox

% Haopeng Hu
% 2018.04.11

% vrep: the vrep object
% clientID: the id of the MATLAB applicaiton
% handels: the set of handles in V-REP scene

disp('Connecting to V-REP service...');
vrep = remApi('remoteApi');     % Use remoteApiProto.m
vrep.simxFinish(-1);            % Stop other connections

% Edit the following parameters for your own application
clientID = vrep.simxStart('127.0.0.1',19997,true,true,6000,5);  % Connect to V-REP
% Attention! port number '19997' is not trivial. It's for the continuous
% operation mode. See remoteApiConnections.txt for details.
% If you want temporary operation mode, change the port number '19997' to 
% a larger one.

if clientID > -1    % Connected!
    disp('Connected!');
    handles = struct('ID',clientID);
else
    disp('Failed connecting to remote API server');
    vrep.delete();
    return;
end
