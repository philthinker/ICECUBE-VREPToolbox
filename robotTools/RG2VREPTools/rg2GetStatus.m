function [ closeSign, gripperSign ] = rg2GetStatus( vrep,clientID )
%rg2GetStatus Get the current status of the RG2
% vrep: the vrep object
% clientID: the client ID
% closeSign: scalar, 1 for close while 0 for open
% gripperSign: scalar, 1 for gripped while 0 for vacant

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.05.15

[~, closeSign] = vrep.simxGetIntegerSignal(clientID,'RG2CLOSED',vrep.simx_opmode_blocking);
[~,gripperSign] = vrep.simxGetIntegerSignal(clientID,'RG2GRASP',vrep.simx_opmode_blocking);


end

