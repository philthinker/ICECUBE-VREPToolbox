function [ ticks ] = wait( obj )
%wait Wait until the task is done
% ticks: The number of ticks counted

% @ICECUBE

j = 0; signal = 99;
while j <= obj.TIMEOUT && signal ~= 0
    j = j+1;
    [res, signal] = obj.vrep.simxGetIntegerSignal(obj.clientID, 'ICECUBE_0', obj.vrep.simx_opmode_blocking);
    obj.vrchk(res);
    pause(obj.step);
end
ticks = j;

end

