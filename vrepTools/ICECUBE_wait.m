function [ ticks ] = ICECUBE_wait( vrep, clientID, tick_step, TIMEOUT )
%ICECUBE_wait ICECUBE Communication Protocol v1.0 - Wait for RML motions
%vrep: The V-REP object
%clientIF: The client ID of the MATLAB application
%tick_step: The time step
%TIMEOUT: The timeout
%ticks: The number of ticks counted

j = 0; signal = 99;
while j<=TIMEOUT && signal ~= 0
    j = j+1;
    [res, signal] = vrep.simxGetIntegerSignal(clientID, 'ICECUBE_0', vrep.simx_opmode_blocking);
    vrchk(vrep,res);
    pause(tick_step);
end
ticks = j;

end

