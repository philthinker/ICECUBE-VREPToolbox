function [ res ] = sim_rg2Action( vrep,clientID,gripperSign, pauseTime )
%sim_rg2Action Drive the RG2 to close or open
% vrep: the vrep object
% clientID: the MATLAB client ID
% gripperSign: 1 for close while 0 for open
% pauseTime: the time to pause
% res: the return code, you can use 'vrepTools\vrchk' to check it

if gripperSign == 1
    % Close
    res = sim_rg2Close(vrep,clientID,vrep.simx_opmode_blocking);
elseif gripperSign == 0
    % Open
    res = sim_rg2Open(vrep,clientID,vrep.simx_opmode_blocking);
end
pause(pauseTime);

end

