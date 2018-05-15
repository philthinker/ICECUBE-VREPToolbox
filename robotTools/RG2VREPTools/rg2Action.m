function [ res ] = rg2Action( vrep,clientID,gripperSign, pauseTime )
%rg2Action Open or Close the RG2 in the vrep sence
% vrep: the vrep object
% clientID: the client ID
% gripperSign: scalar, 1 for close while 0 for open
% pauseTime: the time to pause (not necessary)
% res: the returned code

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.05.15

if nargin < 4
    pauseTime = 2.5;
end

if gripperSign == 1
    % Close
    res = vrep.simxSetIntegerSignal(clientID,'RG2CMD',1,vrep.simx_opmode_blocking);
elseif gripperSign == 0
    % Open
    res = vrep.simxSetIntegerSignal(clientID,'RG2CMD',0,vrep.simx_opmode_blocking);
end
pause(pauseTime);

end

