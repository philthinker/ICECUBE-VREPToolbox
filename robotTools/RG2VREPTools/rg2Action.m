function [ res ] = rg2Action( icecube,gripperSign )
%rg2Action Open or Close the RG2 in the vrep sence
% icecube: the icecube object
% gripperSign: gripperSign, true for close while false for open
% res: the returned code

% ICECUBE Communication Protocol v2.0
% Haopeng Hu
% 2018.07.05

if gripperSign
    % Close
    res = icecube.vrep.simxSetIntegerSignal(icecube.clientID,'RG2CMD',1,icecube.vrep.simx_opmode_blocking);
else
    % Open
    res = icecube.vrep.simxSetIntegerSignal(icecube.clientID,'RG2CMD',0,icecube.vrep.simx_opmode_blocking);
end
icecube.vrchk(res);
pause(2);


end

