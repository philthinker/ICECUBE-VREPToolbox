function [ res ] = start( obj )
%start Start a simulation
% res; the returned code (You can check it by vrchk())

% @ICECUBE

% ICECUBE Communication Protocol v2.0
res = obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_blocking);
obj.vrchk(res);

i=0; ur5ready = 0;
while i < obj.TIMEOUT && ur5ready == 0
    i = i + 1;
    [res, ur5ready] = obj.vrep.simxGetIntegerSignal(obj.clientID,'UR5READY',obj.vrep.simx_opmode_blocking);
    obj.vrchk(res);
    pause(obj.step);
end
if i>= obj.TIMEOUT
    disp('An error occurred in the V-REP server');
    obj.vrep.simxFinish(obj.clientID);
    obj.vrep.delete();
    res = obj.vrep.simx_error_timeout_flag;
else
%     disp('V-REP is started.');
    res = obj.vrep.simx_error_noerror;
end

end

