function [ ticks ] = ICECUBE_wait( vrep, clientID, tick_step, TIMEOUT )
%ICECUBE_wait ICECUBE Communication Protocol v1.0 - Wait for RML motions
%vrep: The V-REP object
%clientIF: The client ID of the MATLAB application
%tick_step: The time step
%TIMEOUT: The timeout
%ticks: The number of ticks counted

i = 0;
while i<=TIMEOUT && vrep.simxGetIntegerSignal(clientID, 'ICECUBE_0', vrep.simx_opmode_blocking) ~= 0
    i = i+1;
    pause(tick_step);
end
ticks = i;

end

