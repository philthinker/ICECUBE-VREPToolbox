function [ res ] = ICECUBE_stop( vrep, clientID )
%ICECUBE_stop Stop the simulation
%vrep: The V-REP object
%clientID: The ID of the MALTAB client
%res: The result

% ICECUBE Communication Protocol v2.0
disp('Stop the simulation ...');
res = vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 01, vrep.simx_opmode_blocking);
pause(0.5);

end

