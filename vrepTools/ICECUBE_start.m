function [ res ] = ICECUBE_start( vrep, clientID, step, TIMEOUT )
%ICECUBE_start Start your V-REP simulation
%vrep: The V-REP object
%clientID: The ID of the MALTAB client
%step: The time step
%TIMEOUT: The timeout
%res: The result

% ICECUBE Communication Protocol v1.0
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('Simulation Started!');
vrchk(vrep,res);
i=0; ClientRunning = 0;
while i < TIMEOUT && ClientRunning == 0
    i = i + 1;
    [res, ClientRunning] = vrep.simxGetIntegerSignal(clientID,'ClientRunning',vrep.simx_opmode_blocking);
    vrchk(vrep,res);
    pause(step);
end
if i>= TIMEOUT
    disp('An error occurred in your V-REP server');
    vrep.simxFinish(clientID);
    vrep.delete();
    res = vrep.simx_error_timeout_flag;
else
    disp('V-REP is started.');
    res = vrep.simx_error_noerror;
end

end

