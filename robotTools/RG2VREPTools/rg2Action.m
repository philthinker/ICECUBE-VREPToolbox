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

%% ICECUBE v2.2
% % [ res ] = rg2Action( vrep,clientID,gripperSign )
% if gripperSign == 1
%     % Close
%     res = vrep.simxSetIntegerSignal(clientID,'RG2CMD',1,vrep.simx_opmode_blocking);
% elseif gripperSign == 0
%     % Open
%     res = vrep.simxSetIntegerSignal(clientID,'RG2CMD',0,vrep.simx_opmode_blocking);
% end
% pause(2);

end

