function [ res ] = stop( obj )
%stop Stop the running simulation
% res; the returned code (You can check it by vrchk())

% @ICECUBE

% ICECUBE Communication Protocol v2.0
% disp('Stop the simulation ...');
res = obj.vrep.simxSetIntegerSignal(obj.clientID, 'ICECUBE_0', 01, obj.vrep.simx_opmode_blocking);
pause(0.5);


end

