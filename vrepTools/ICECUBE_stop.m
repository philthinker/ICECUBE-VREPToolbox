function [ res ] = ICECUBE_stop( vrep, clientID )
%ICECUBE_stop Stop the simulation
%vrep: The V-REP object
%clientID: The ID of the MALTAB client
%res: The result

% ICECUBE Communication Protocol v1.0
res = vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 3, vrep.simx_opmode_blocking);

end

