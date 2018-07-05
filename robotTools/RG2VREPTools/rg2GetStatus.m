function [ closeSign, gripperSign ] = rg2GetStatus( icecube )
%rg2GetStatus Get the current status of the RG2
% icecube: the icecube object
% closeSign: scalar, 1 for close while 0 for open
% gripperSign: scalar, 1 for gripped while 0 for vacant

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.07.05

[~, closeSign] = icecube.vrep.simxGetIntegerSignal(icecube.clientID,'RG2CLOSED',icecube.vrep.simx_opmode_blocking);
[~,gripperSign] = icecube.vrep.simxGetIntegerSignal(icecube.clientID,'RG2GRASP',icecube.vrep.simx_opmode_blocking);

%% ICECUBE v2.2
% [ closeSign, gripperSign ] = rg2GetStatus( vrep,clientID )
% [~, closeSign] = vrep.simxGetIntegerSignal(clientID,'RG2CLOSED',vrep.simx_opmode_blocking);
% [~,gripperSign] = vrep.simxGetIntegerSignal(clientID,'RG2GRASP',vrep.simx_opmode_blocking);


end

